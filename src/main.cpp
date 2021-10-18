#include "mbed.h"
#include "ros.h"
#include "std_msgs/Float32.h"

#define SPIN_DELAY_MS 10U
#define SENSOR_INTER_MEASUREMENT_MS 1000U
#define MOCK_PIN ADC_TEMP

class RosMockSensorTest : public ros::NodeHandle, private NonCopyable<RosMockSensorTest>
{
public:
    /**
     * @brief Initialise ros mock sensor node.
     */
    void init()
    {
        // initialise ros node
        initNode();

        // prepare sensor data
        sensor_msg.data = 0.0f;

        // advertise publisher
        advertise(sensor_pub);

        // set LEDs states
        led1 = 0;
        led2 = 1;
    };

    /**
     * @brief Run ros mock sensor node test.
     */
    void spin()
    {
        uint64_t last_publish_time = 0;

        while (1)
        {
            uint64_t time = Kernel::get_ms_count();

            if (time - last_publish_time >= SENSOR_INTER_MEASUREMENT_MS)
            {
                // signal with LEDs
                led1 = !led1;
                led2 = !led2;

                // read mock sensor data
                sensor_msg.data = mock_sensor.read();

                // write log message
                loginfo("Publishing a new sensor reading...");

                // publish ros message
                sensor_pub.publish(&sensor_msg);

                last_publish_time = time;
            }

            spinOnce();

            ThisThread::sleep_until(time + SPIN_DELAY_MS);
        }
    }

private:
    DigitalOut led1{LED1, 0};
    DigitalOut led2{LED2, 0};
    AnalogIn mock_sensor{MOCK_PIN};
    std_msgs::Float32 sensor_msg;
    ros::Publisher sensor_pub{"mock_sensor", &sensor_msg};
};

int main()
{
    RosMockSensorTest mock_sensor;
    mock_sensor.init();
    mock_sensor.spin();
}