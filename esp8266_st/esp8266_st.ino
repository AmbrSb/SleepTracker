#include <Scheduler.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <deque>


#define COOP
#define SERIAL_BAUD_RATE 921600
#define DEBUG_MEASURE
#define DEBUG_NET

#define WIFI_SSID     "SSID"
#define WIFI_PASSWORD "PASSWORD"

#ifndef COOP
#define CRIT_SECTION_START unsigned long _i_flags = arch_local_irq_save()
#define CRIT_SECTION_END   arch_local_irq_restore(_i_flags)
#else
/**
 * If the system is working under a cooperative multi-tasking regimen,
 * then we can forgo disabling of interrupts in certain critical sections.
 */
#define CRIT_SECTION_START
#define CRIT_SECTION_END
#endif

static uint8_t log_level = 5;

#define st_debug_print(lvl, _fs)    \
  do {                              \
    if (log_level >= lvl)           \
      Serial.println((_fs));        \
  } while (0)

/**
 * Disable interrupts
 */
static inline unsigned long arch_local_irq_save(void)
{
    /**
     * Use RSIL to set interrupt level to 1 and store the current value in flags
     */
    unsigned long ps;
    asm volatile("rsil %0, 1" : "=a" (ps) :: "memory");
    return ps;
}

/**
 * Restore interrupt level in PS register
 */
static inline void arch_local_irq_restore(unsigned long flags)
{
    asm volatile("wsr %0, ps; rsync"
                 :: "a" (flags) : "memory");
}

/**
 * Data sources will generated instances of orientation which will be
 * serialized and sent to client(s) over wifi.
 */
struct orientation {
    /**
     * Unit of x, y, z is g (i.e 9.8 m2/s). These are measurements of
     * acceleration along each axis.
     */
    float x, y, z;
    float roll, pitch;

    void dump()
    {
        char str[64];
        snprintf(str, sizeof(str), "%f %f %f %f %f",
                 x, y, z, roll, pitch);
        st_debug_print(1, str);
    }

    template <uint16_t N>
    void dump(char buffer[N])
    {
      
        snprintf(buffer, N, "%f %f %f %f %f",
                 x, y, z, roll, pitch);
    }
};

#define ADXL345_POWER_CTL_REG    0x2D
#define ADXL345_RESOLUTION_FULL  0x8
#define ADXL345_RESOLUTION_2G    0x0
#define ADXL345_RESOLUTION_4G    0x1
#define ADXL345_RESOLUTION_8G    0x2
#define ADXL345_RESOLUTION_16G   0x3

/**
 * This class represents an ADXL345 accelerometer device.
 * 
 * @tparam DEV is I2C bus address of the device.
 * @tpraam RESOLUTION indicates the the range/resolution mode of the
 *         device that this instance of the class will operate.
 */
template <uint8_t DEV = 0x53, uint8_t RESOLUTION = ADXL345_RESOLUTION_2G>
class adxl345 {
public:
    adxl345()
      : kMult{RESOLUTION == ADXL345_RESOLUTION_FULL ? 256.0 : 256.0 / pow(2, RESOLUTION)}
    {
        this->write(DEV, ADXL345_POWER_CTL_REG, 0);
        this->write(DEV, ADXL345_POWER_CTL_REG, 16);
        this->write(DEV, ADXL345_POWER_CTL_REG, 8);
    }

    /**
     * Read the oreientation data from the device directly.
     */
    orientation
    read_orientation()
    {
        orientation ornt;
        byte data[6];
        int16_t xi, yi, zi;

        this->write(DEV, 0x31, RESOLUTION);

        this->read(DEV, 0x32, 6, data);
        xi = (int16_t)(data[1] << 8) | data[0];
        yi = (int16_t)(data[3] << 8) | data[2];
        zi = (int16_t)(data[5] << 8) | data[4];

        ornt.x = xi / kMult;
        ornt.y = yi / kMult;
        ornt.z = zi / kMult;
        ornt.roll = calc_roll(xi, yi, zi);
        ornt.pitch = calc_pitch(xi, yi, zi);

        return (ornt);
    }

private:
    /**
     * All accelration readings will be devided by mMult which is
     * calculated based on table 1 of ADXL345 datasheet on page 5.
     * https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
     */
    const float kMult;
    /**
     * Read 'num' bytes from 'addr' on device 'dev' in the array 'data'
     */
    void read(int dev, byte addr, int num, byte data[])
    {
        Wire.beginTransmission(dev);
        Wire.write(addr);
        Wire.endTransmission();

        Wire.beginTransmission(dev);
        Wire.requestFrom(dev, num);
        for (int i = 0; i < num; i++) {
            if(!Wire.available())
                break;
            data[i] = Wire.read();
        }
        Wire.endTransmission();
    }
     
    /**
     * Write the value 'val' into address 'addr' on device 'dev'
     */
    void write(int dev, byte addr, byte val)
    {
        Wire.beginTransmission(dev);
        Wire.write(addr);
        Wire.write(val);
        Wire.endTransmission();
    }

    float
    calc_roll(int16_t x, int16_t y, int16_t z)
    {
        float roll;
        roll = atan2(y , z) * 57.3;
        return (roll);
    }

    float
    calc_pitch(int16_t x, int16_t y, int16_t z)
    {
        float pitch;
        pitch = atan2((-x) , sqrt(y * y + z * z)) * 57.3;
        return (pitch);
    }
};
#undef ADXL345_POWER_CTL_REG

class DataSourceTask {
public:
    virtual bool pop(orientation &ornt) = 0;
};

/**
 * Task responsible for collecting accelerometer readings and storing
 * them to be later read by the DataService task to be sent over the
 * network.
 * 
 * @tparam FREQ How many times per second should we collect sensor readings.
 * @tparam RESOLUTION The measurement resolution/sensitivity to set for
 *         the accelerometer.
 */
template <uint16_t FREQ, uint8_t RESOLUTION>
class MeasureTask: public Task, public DataSourceTask {
public:
    bool
    pop(orientation &ornt) override
    {
        if (queue.size() == 0)
          return false;

        CRIT_SECTION_START;
        ornt = queue.front();
        queue.pop_front();
        CRIT_SECTION_END;
        return true;
    }

protected:
    void setup()
    {
        Wire.begin();
    }
     
    void loop()
    {
        adxl345<0x53, RESOLUTION> dev;
        orientation ornt = dev.read_orientation();
        enque(ornt);
#ifdef DEBUG_MEASURE
        ornt.dump();
#endif
        delay(1000 / FREQ);
    }

private:
    /**
     * This small queue is intended to minimize data loss in case of
     * a temporary network failure. All measurements are buffered in
     * this queue to be later read by the network service task.
     * If maximum queue size is exceeded, we gradually discard old
     * data. Currently with a max size of 1024 we should be able to
     * tolerate network failures of up to 10 seconds.
     */
    std::deque<orientation> queue;
    const uint16_t max_queue_sz = 1024;
    uint16_t queue_sz = 0;

    void enque(orientation ornt)
    {
        CRIT_SECTION_START;
        if (queue_sz >= max_queue_sz)
            discard(1);
        queue.push_back(ornt);
        queue_sz++;
        CRIT_SECTION_END;
    }

    /**
     * Discard 'num' elements from the end of the queue
     */
    void
    discard(uint16_t num)
    {
        CRIT_SECTION_START;
        queue.erase(queue.begin(), queue.begin() + num);
        queue_sz -= num;
        CRIT_SECTION_END;
    }
};

/**
 * A network data sink task that takes data from a instance
 * of Data Source and sends over a tcp connection to
 * the connected client.
 */
class NetworkTask: public Task {
public:

    NetworkTask(std::vector<DataSourceTask*> data_sources, char const *ssid, char const *password)
      : data_sources{data_sources},
        ssid{ssid},
        password{password}
    {
    }

    void setup()
    {
        WiFi.begin(ssid, password);
        wifiServer = std::make_shared<WiFiServer>(80);

       
        while (WiFi.status() != WL_CONNECTED) {
          delay(2000);
          st_debug_print(2, "Connecting...");
        }
       
        st_debug_print(2, "Connected to WiFi. IP:");
        st_debug_print(2, WiFi.localIP());
       
        wifiServer->begin();      
    }

    /**
     * Verify that the current client is OK, or otherwise
     * try to acquire a new pending client from the wifi
     * server and return true. Otherwise, return false, to
     * indicate that there is no reachable client at the
     * moment.
     */
    bool
    get_client()
    {
        if (client && client.connected()) {
            /**
             * Current client instance is OK
             */
            return true;
        }

        client = wifiServer->available();
        if (!client)
          return false;

        st_debug_print(2, "Client connected");
        return true;
    }

    void
    loop()
    {
        orientation item;
        char buffer[64];

        if (!get_client()) {
            st_debug_print(2, "No client");
            delay(300);
            return;
        }        

        while (client) {
            for (auto data_source: data_sources) {
              while (data_source->pop(item)) {
                  /**
                   * While there is data keep sending to the client
                   * non-stop.
                   * This loop is based on the underlying assumption
                   * that that network bandwidth is considerably higher
                   * than the volume of data being generated by sensor
                   * tasks. Otherwise this loop will block sensor tasks.
                   */
                  item.dump<sizeof(buffer)>(buffer);
                  client.println(buffer);
              }
            }
            /**
             * Give up turn to scheduler so sensor tasks
             * can do their job.
             */
            delay (10);
        }
    }

private:
    std::vector<DataSourceTask*> data_sources;
    const char* ssid;
    const char* password;
    std::shared_ptr<WiFiServer> wifiServer;
    WiFiClient client;
};


/**
 * Definition of scheduler tasks and their respective types.
 * Currently we have 3 measurement tasks:
 *   full resultion
 *   +/- 4g
 *   +/- 16g
 * Finally, there is one newtwork task that is responsible for
 * draining data generated by the measurement tasks.
 */
constexpr int kFreq = 10;
using MT_Full = MeasureTask<kFreq, ADXL345_RESOLUTION_FULL>;
using MT_4G   = MeasureTask<kFreq, ADXL345_RESOLUTION_4G>;
using MT_16G  = MeasureTask<kFreq, ADXL345_RESOLUTION_16G>;

MT_Full measure_task_full;
MT_4G   measure_task_4g;
MT_16G  measure_task_16g;
std::vector<DataSourceTask*> data_source_tasks = {&measure_task_full,
                                                  &measure_task_4g,
                                                  &measure_task_16g};
NetworkTask network_task{data_source_tasks, WIFI_SSID, WIFI_PASSWORD};

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    Scheduler.start(&measure_task_full);
    Scheduler.start(&measure_task_4g);
    Scheduler.start(&measure_task_16g);
    Scheduler.start(&network_task);
    Scheduler.begin();
    __builtin_unreachable();
}

void loop()
{
    __builtin_unreachable();
}
