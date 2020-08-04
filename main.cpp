/////////////////////////////////////////////////////////////////////////////
// -------------------- DOT LIBRARY REQUIRED ------------------------------//
// * https://developer.mbed.org/teams/MultiTech/code/libmDot-dev-mbed5/    //
// * https://developer.mbed.org/teams/MultiTech/code/libmDot-mbed5/        //
/////////////////////////////////////////////////////////////////////////////

#include "mbed.h"
#include "dot_util.h"
#include "RadioEvent.h"
#include "Adafruit_SI1145.h"

/////////////////////////////////////////////////////////////
//                  Network setup                          //
/////////////////////////////////////////////////////////////
static std::string network_name = "UQ_St_Lucia";
static std::string network_passphrase = "L0raStLucia";
static uint8_t network_id[] = { 0x6C, 0x4E, 0xEF, 0x66, 0xF4, 0x79, 0x86, 0xA6 };
static uint8_t network_key[] = { 0x1F, 0x33, 0xA1, 0x70, 0xA5, 0xF1, 0xFD, 0xA0, 0xAB, 0x69, 0x7A, 0xAE, 0x2B, 0x95, 0x91, 0x6B };
static uint8_t frequency_sub_band = 7;
static bool public_network = false;
static uint8_t ack = 1;
static bool adr = true;

// Initialise mDot
mDot* dot = NULL;

// Initialise LoRaWAN channel plan
lora::ChannelPlan* plan = NULL;

// Initialise UART for debugging
Serial pc(USBTX, USBRX);

// Initialise relay output pins
DigitalOut led1(PB_1);
DigitalOut led2(PB_0);
DigitalOut led3(PA_5);
DigitalOut led4(PA_7);
DigitalOut led5(PA_4);

// Initialise sensor
static Adafruit_SI1145 uv = Adafruit_SI1145();

// Update the LEDs based on the uv index
void UpdateLEDs(uint8_t uv_index)
{
    if (uv_index == 0) {
//        logInfo("LEDs off");

        led1 = 0;
        led2 = 0;
        led3 = 0;
        led4 = 0;
        
        led5 = 0;

    } else if (uv_index >= 1 && uv_index <= 2) {
//        logInfo("led1");

        led2 = 0;
        led3 = 0;
        led4 = 0;
        led5 = 0;

        led1 = 1;

    } else if (uv_index >= 3 && uv_index <= 5) {
//        logInfo("led2");
        led1 = 0;
        led3 = 0;
        led4 = 0;
        led5 = 0;

        led2 = 1;

    } else if (uv_index >= 6 && uv_index <= 7) {
//        logInfo("led3");
        led1 = 0;
        led2 = 0;
        led4 = 0;
        led5 = 0;

        led3 = 1;

    } else if (uv_index >= 8 && uv_index <= 10) {
//        logInfo("led4");
        led1 = 0;
        led2 = 0;
        led3 = 0;
        led5 = 0;

        led4 = 1;

    } else if (uv_index >= 11) {
//        logInfo("led5");
        led1 = 0;
        led2 = 0;
        led3 = 0;
        led4 = 0;

        led5 = 1;
    }
}

// Add latest uv index into the rolling window
void updateLastTwenty(float current_index, float *last_twenty)
{
    for (uint8_t i = 19; i > 0; i-- ) {
        last_twenty[i] = last_twenty[i-1];
    }
    last_twenty[0] = current_index;
}

// Compute the average of the uv index window
float averageIndex(float * last_twenty)
{
    float ret, sum = 0;

    for (uint8_t i = 0; i < 20; i++) {
        sum += last_twenty[i];
    }
    ret = sum/20;
    return ret;
}



int main()
{
    // Custom event handler for automatically displaying RX data
    RadioEvent events;

    pc.baud(9600);

    mts::MTSLog::setLogLevel(mts::MTSLog::TRACE_LEVEL);

    plan = new lora::ChannelPlan_AU915();

    assert(plan);

    dot = mDot::getInstance(plan);
    assert(dot);

    // attach the custom events handler
    dot->setEvents(&events);

    if (!dot->getStandbyFlag()) {
        logInfo("mbed-os library version: %d", MBED_LIBRARY_VERSION);

        // start from a well-known state
        logInfo("defaulting Dot configuration");
        dot->resetConfig();
        dot->resetNetworkSession();

        // make sure library logging is turned on
        dot->setLogLevel(mts::MTSLog::INFO_LEVEL);

        // update configuration if necessary
        if (dot->getJoinMode() != mDot::OTA) {
            logInfo("changing network join mode to OTA");
            if (dot->setJoinMode(mDot::OTA) != mDot::MDOT_OK) {
                logError("failed to set network join mode to OTA");
            }
        }
        // in OTA and AUTO_OTA join modes, the credentials can be passed to the library as a name and passphrase or an ID and KEY
        // only one method or the other should be used!
        // network ID = crc64(network name)
        // network KEY = cmac(network passphrase)
        update_ota_config_name_phrase(network_name, network_passphrase, frequency_sub_band, public_network, ack);

        // enable or disable Adaptive Data Rate
        dot->setAdr(adr);
        
        // Set maximum transmit power
        printf("setting TX power\r\n");
        if((dot->setTxPower(20)) != mDot::MDOT_OK) {
            logInfo("falled to set Tx Power");
        }

        // Set maximum number of retries
        printf("setting number of retries\r\n");
        if((dot->setAck(8)) != mDot::MDOT_OK) {
            logInfo("falled to set number of retries");
        }

        // save changes to configuration
        logInfo("saving configuration");
        if (!dot->saveConfig()) {
            logError("failed to save configuration");
        }

        // display configuration (for debugging)
        display_config();
    } else {
        // restore the saved session if the dot woke from deepsleep mode
        // useful to use with deepsleep because session info is otherwise lost when the dot enters deepsleep
        logInfo("restoring network session from NVM");
        dot->restoreNetworkSession();
    }

    float average_index, current_index, last_twenty[20] = {0};
    uint8_t rounded_index = 0, sensor_timeout, send_counter = 0;
    std::vector<uint8_t> tx_data;
    
    // Create LoRaWAN UV index message header
    // "sc/u/{index}"
    tx_data.push_back('s');
    tx_data.push_back('c');
    tx_data.push_back('/');
    tx_data.push_back('u');
    tx_data.push_back('/');
    tx_data.push_back('0');
    tx_data.push_back('0' + rounded_index);

    // Turn off LEDs
    UpdateLEDs(0);

    while (true) {

        // if sensor not connected after 100 trys send help message
        sensor_timeout = 0;
        while (! uv.beginDefault()) {
            sensor_timeout++;

            // Sensor not recognised after 100 trys
            if (sensor_timeout > 100) {

//                logInfo("Sensor Not Recognised");

                // Turn off all LEDs
                UpdateLEDs(0);

                // Send error message
                // "sc/t/u/1"
                std::vector<uint8_t> sensor_disconnected;
                sensor_disconnected.push_back('s');
                sensor_disconnected.push_back('c');
                sensor_disconnected.push_back('/');
                sensor_disconnected.push_back('t');
                sensor_disconnected.push_back('/');
                sensor_disconnected.push_back('u');
                sensor_disconnected.push_back('/');
                sensor_disconnected.push_back('1');

                //join network if not joined
                if (!dot->getNetworkJoinStatus()) {
                    while ( join_network() != mDot::MDOT_OK);
                }

                send_data(sensor_disconnected);
        
                // Reset send_counter and keep trying to connect
                sensor_timeout = 0;
                while (! uv.beginDefault()) {
                    wait(0.1);
                }
                
                // If manage to conect, send a message that sensor connected again
                std::vector<uint8_t> sensor_connected;
                sensor_connected.push_back('s');
                sensor_connected.push_back('c');
                sensor_connected.push_back('/');
                sensor_connected.push_back('t');
                sensor_connected.push_back('/');
                sensor_connected.push_back('u');
                sensor_connected.push_back('/');
                sensor_connected.push_back('0');
                
                //join network if not joined
                if (!dot->getNetworkJoinStatus()) {
                    while ( join_network() != mDot::MDOT_OK);
                }

                send_data(sensor_connected);
            }
            wait(1);

        }

        // UV index returned from sensor is scaled by factor of 100
        current_index = uv.readUV() / 100.0;
        
        // Less than 0.3 will be at night, greater can occur at day
        if (current_index < 1) {
            if (current_index > 0.3) {
                current_index = 1;
            }
        }

        // Add new UV index into array
        updateLastTwenty(current_index, last_twenty);

        // Average of array 20 indices updated every 6 seconds
        // Thus, average of 2 minute window
        average_index = averageIndex(last_twenty);

        // add 0.5 because c++ truncates (rounds down)
        rounded_index = (uint8_t)(average_index + 0.5);

//        logInfo("from sensor: %f, average: %f, rounded: %d", current_index, average_index, rounded_index);

        UpdateLEDs(rounded_index);
        
        // only send data every 30 seconds
        if (send_counter == 0) {
            
            //join network if not joined
            if (!dot->getNetworkJoinStatus()) {
                if ( join_network() == mDot::MDOT_OK) {
    
                    // Logic to send single or double digit UV index
                    tx_data[6] = '0' + (rounded_index % 10);
                    tx_data[5] = '0' + (uint8_t)(rounded_index / 10);
    
                    send_data(tx_data);
    
                }
            } else {
    
                // Logic to send single or double digit UV index
                tx_data[6] = '0' + (rounded_index % 10);
                tx_data[5] = '0' + (uint8_t)(rounded_index / 10);
    
                send_data(tx_data);
    
            }
        }
        
        // This value will become 0 every 6 loop repeats (30 seconds)
        send_counter = (send_counter + 1)%5;

        wait(6);

    }

    return 0;
}