
#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* Common addresses definition for temperature sensor. */
#define HTU21D_ADDR      0x40
#define HTU21D_TEMP      0xF3
#define HTU21D_HUMI      0xF5


/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static int temp;
static int humi;
static int flag = 0;//1->permettre TX 2->permettre RX

/**
 * @brief Function for setting active mode on MMA7660 accelerometer.
 */
void HTU21D_mesure_temp()
{
		NRF_LOG_INFO("send command temp");
    ret_code_t err_code;
    /* Writing to pointer byte. */
		m_xfer_done = false;
    uint8_t reg = HTU21D_TEMP;
    err_code = nrf_drv_twi_tx(&m_twi, HTU21D_ADDR, &reg, 1, false);
    APP_ERROR_CHECK(err_code);
}

void HTU21D_mesure_humi()
{
		NRF_LOG_INFO("send command humi");
    ret_code_t err_code;
    /* Writing to pointer byte. */
		m_xfer_done = false;
    uint8_t reg = HTU21D_HUMI;
    err_code = nrf_drv_twi_tx(&m_twi, HTU21D_ADDR, &reg, 1, false);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(int *temp)
{
    NRF_LOG_INFO("Temperature: %d Celsius degrees.", &temp);
}


void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //data_handler(&temp);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/*** @brief UART initialization */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_HTU21D_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

   
		
		do
    {
       
        NRF_LOG_INFO("initialization");
				err_code = nrf_drv_twi_init(&m_twi, &twi_HTU21D_config, twi_handler, NULL); 
				APP_ERROR_CHECK(err_code);
        if(err_code == NRF_SUCCESS)
        {
            NRF_LOG_INFO("initialization success");
					
        }
        nrf_drv_twi_enable(&m_twi);
				
    }while(0);
}

/** * @brief Function for reading data from temperature sensor.*/
static void read_sensor_data_temp()
{
		m_xfer_done = false;
		NRF_LOG_INFO("-----------------------------Receive temperature-----------------------------");
		uint8_t m_sample[3] = {0,0,0};
    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, HTU21D_ADDR, m_sample, 3);
		do
        {
            __WFE();
        }while (m_xfer_done == false);
				
		NRF_LOG_INFO("%x",m_sample[0]);
		NRF_LOG_INFO("%x",m_sample[1]);
		NRF_LOG_INFO("%x",m_sample[2]);
		
		uint16_t rawTemperature = (m_sample[0] << 8) | m_sample[1];
		int raw = (int)rawTemperature;
		NRF_LOG_INFO("%x",rawTemperature);
		NRF_LOG_INFO("%d",raw);

		float raw1 = (float)raw;
		//NRF_LOG_INFO("My float number: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(raw1));
				
    float tempTemperature =  raw1 / (float)65536; //2^16 = 65536
		//NRF_LOG_INFO("My float number: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(tempTemperature));
				
    float rh = -46.85 + 175.72 * tempTemperature; 
		//NRF_LOG_INFO("My float number: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(rh));
				
    temp = (int)rh; 
		NRF_LOG_INFO("Temperature: %d Celsius degrees.", temp);

    APP_ERROR_CHECK(err_code);
}



static void read_sensor_data_humi()
{
		m_xfer_done = false;
		NRF_LOG_INFO("-------------------------------Receive humitity-------------------------------");
		uint8_t m_sample[3] = {0,0,0};
    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, HTU21D_ADDR, m_sample, 3);
		do
        {
            __WFE();
        }while (m_xfer_done == false);
				
		NRF_LOG_INFO("%x",m_sample[0]);
		NRF_LOG_INFO("%x",m_sample[1]);
		NRF_LOG_INFO("%x",m_sample[2]);
	
		uint16_t rawHumitite = (m_sample[0] << 8) | m_sample[1];
    float tempHumitite =  (float)rawHumitite / (float)65536; //2^16 = 65536
    float rh = (float)-6 + ((float)125 *(float)tempHumitite); 
    humi = (int)rh; 
		NRF_LOG_INFO("Humitity: %d percent.", humi);

    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
		
		NRF_LOG_INFO("---------------------------------Communication---------------------------------");
    NRF_LOG_FLUSH();
    twi_init();
		flag = 1;
		int i = 0;
    while (i<1)
    {
        HTU21D_mesure_temp();
				do
        {
            __WFE();
        }while (m_xfer_done == false);
				nrf_delay_ms(500);
				
				read_sensor_data_temp();
				
				nrf_delay_ms(500);
				
				HTU21D_mesure_humi();
				do
        {
            __WFE();
        }while (m_xfer_done == false);
				nrf_delay_ms(500);
				
				read_sensor_data_humi();
				
				i++;
        NRF_LOG_FLUSH();
    }
}

/** @} */
