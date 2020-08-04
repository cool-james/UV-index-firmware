/***************************************************
  This is a library for the Si1145 UV/IR/Visible Light Sensor

  Designed specifically to work with the Si1145 sensor in the
  adafruit shop
  ----> https://www.adafruit.com/products/1777

  These sensors use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_SI1145.h"

I2C i2c(I2C_SDA , I2C_SCL ); //Default frequency is 100kHz

Adafruit_SI1145::Adafruit_SI1145()
{
    _addr = SI1145_ADDR;
}

/***************************************************************************//**
 * @brief
 *   Converts the 12-bit factory test value from the Si114x and returns the
 *   fixed-point representation of this 12-bit factory test value.
 ******************************************************************************/
static uint32_t decode(uint32_t input)
{
    int32_t  exponent, exponent_bias9;
    uint32_t mantissa;

    if(input==0) return 0.0;

    exponent_bias9 = (input & 0x0f00) >> 8;
    exponent       = exponent_bias9 - 9;

    mantissa       = input & 0x00ff; // fraction
    mantissa       |=        0x0100; // add in integer

    // representation in 12 bit integer, 20 bit fraction
    mantissa       = mantissa << (12+exponent);
    return mantissa;
}

/***************************************************************************//**
 * @brief
 *   The buffer[] is assumed to point to a byte array that containst the
 *   factory calibration values after writing 0x12 to the command register
 *   This function takes the 12 bytes from the Si114x, then converts it
 *   to a fixed point representation, with the help of the decode() function
 ******************************************************************************/
static uint32_t collect(uint8_t* buffer,
                        uint8_t msb_addr,
                        uint8_t lsb_addr,
                        uint8_t alignment)
{
    uint16_t value;
    uint8_t  msb_ind = msb_addr - 0x22;
    uint8_t  lsb_ind = lsb_addr - 0x22;

    if(alignment == 0) {
        value =  buffer[msb_ind]<<4;
        value += buffer[lsb_ind]>>4;
    } else {
        value =  buffer[msb_ind]<<8;
        value += buffer[lsb_ind];
        value &= 0x0fff;
    }

    if(   ( value == 0x0fff )
            || ( value == 0x0000 ) ) return FX20_BAD_VALUE;
    else return decode( value );
}

/***************************************************************************//**
 * @brief
 *   This performs a shift_left function. For convenience, a negative
 *   shift value will shift the value right. Value pointed will be
 *   overwritten.
 ******************************************************************************/
static void shift_left(uint32_t* value_p, int8_t shift)
{
    if(shift > 0)
        *value_p = *value_p<<shift ;
    else
        *value_p = *value_p>>(-shift) ;
}

/// @cond DOXYGEN_SHOULD_SKIP_THIS
#define ALIGN_LEFT   1
#define ALIGN_RIGHT -1
/// @endcond
/***************************************************************************//**
 * @brief
 *   Aligns the value pointed by value_p to either the LEFT or RIGHT
 *   the number of shifted bits is returned. The value in value_p is
 *   overwritten.
 ******************************************************************************/
static int8_t align( uint32_t* value_p, int8_t direction )
{
    int8_t   local_shift, shift ;
    uint32_t mask;

    // Check invalid value_p and *value_p, return without shifting if bad.
    if( value_p  == NULL )  return 0;
    if( *value_p == 0 )     return 0;

    // Make sure direction is valid
    switch( direction ) {
        case ALIGN_LEFT:
            local_shift =  1 ;
            mask  = 0x80000000L;
            break;

        case ALIGN_RIGHT:
            local_shift = -1 ;
            mask  = 0x00000001L;
            break;

        default:
            // Invalid direction, return without shifting
            return 0;
    }

    shift = 0;
    while(1) {
        if(*value_p & mask ) break;
        shift++;
        shift_left( value_p, local_shift );
    }
    return shift;
}

/***************************************************************************//**
 * @brief
 * This compile switch used only to experiment with
 * various rounding precisions. The flexibility has
 * a small performance price.
 ******************************************************************************/
#define FORCE_ROUND_16 1

/***************************************************************************//**
 * @brief
 *
 *   The fx20_round was designed to perform rounding to however many significant
 *   digits. However, for the factory calibration code, rounding to 16 always is
 *   sufficient. So, the FORCE_ROUND_16 define is provided just in case it would
 *   be necessary to dynamically choose how many bits to round to.
 *   Rounds the uint32_t value pointed by ptr, by the number of bits
 *   specified by round.
 ******************************************************************************/
static void fx20_round
(
    uint32_t *value_p
#if !FORCE_ROUND_16
    , int8_t round
#endif
)
{
    int8_t  shift;

#if FORCE_ROUND_16
    // Use the following to force round = 16
    uint32_t mask1  = 0xffff8000;
    uint32_t mask2  = 0xffff0000;
    uint32_t lsb    = 0x00008000;
#else
    // Use the following if you want to routine to be
    // capable of rounding to something other than 16.
    uint32_t mask1  = ((2<<(round))-1)<<(31-(round));
    uint32_t mask2  = ((2<<(round-1))-1)<<(31-(round-1));
    uint32_t lsb    = mask1-mask2;
#endif

    shift = align( value_p, ALIGN_LEFT );
    if( ( (*value_p)&mask1 ) == mask1 ) {
        *value_p = 0x80000000;
        shift -= 1;
    } else {
        *value_p += lsb;
        *value_p &= mask2;
    }

    shift_left( value_p, -shift );
}

/***************************************************************************//**
 * @brief
 *   The fx20_divide and fx20_multiply uses this structure to pass
 *   values into it.
 ******************************************************************************/
struct operand_t {
    uint32_t op1;  /**< Operand 1 */
    uint32_t op2;  /**< Operand 2 */
};

/***************************************************************************//**
 * @brief
 *   Returns a fixed-point (20-bit fraction) after dividing op1/op2
 ******************************************************************************/
static uint32_t fx20_divide( struct operand_t* operand_p )
{
    int8_t    numerator_sh=0, denominator_sh=0;
    uint32_t  result;
    uint32_t* numerator_p;
    uint32_t* denominator_p;

    if( operand_p == NULL ) return FX20_BAD_VALUE;

    numerator_p   = &operand_p->op1;
    denominator_p = &operand_p->op2;

    if(   (*numerator_p   == FX20_BAD_VALUE)
            || (*denominator_p == FX20_BAD_VALUE)
            || (*denominator_p == 0             ) ) return FX20_BAD_VALUE;

    fx20_round  ( numerator_p   );
    fx20_round  ( denominator_p );
    numerator_sh   = align ( numerator_p,   ALIGN_LEFT  );
    denominator_sh = align ( denominator_p, ALIGN_RIGHT );

    result = *numerator_p / ( (uint16_t)(*denominator_p) );
    shift_left( &result , 20-numerator_sh-denominator_sh );

    return result;
}

/***************************************************************************//**
 * @brief
 *   Returns a fixed-point (20-bit fraction) after multiplying op1*op2
 ******************************************************************************/
static uint32_t fx20_multiply( struct operand_t* operand_p )
{
    uint32_t  result;
    int8_t    val1_sh, val2_sh;
    uint32_t* val1_p;
    uint32_t* val2_p;

    if( operand_p == NULL ) return FX20_BAD_VALUE;

    val1_p = &(operand_p->op1);
    val2_p = &(operand_p->op2);

    fx20_round( val1_p );
    fx20_round( val2_p );

    val1_sh = align( val1_p, ALIGN_RIGHT );
    val2_sh = align( val2_p, ALIGN_RIGHT );


    result = (uint32_t)( ( (uint32_t)(*val1_p) ) * ( (uint32_t)(*val2_p) ) );
    shift_left( &result, -20+val1_sh+val2_sh );

    return result;
}


/***************************************************************************//**
 * @brief
 *   Structure Definition for calref array
 ******************************************************************************/
struct cal_ref_t {
    uint32_t sirpd_adchi_irled; /**< Small IR PD gain, IR LED, Hi ADC Range     */
    uint32_t sirpd_adclo_irled; /**< Small IR PD gain, IR LED, Lo ADC Range     */
    uint32_t sirpd_adclo_whled; /**< Small IR PD gain, White LED, Lo ADC Range  */
    uint32_t vispd_adchi_whled; /**< VIS PD gain, White LED, Lo ADC Range       */
    uint32_t vispd_adclo_whled; /**< VIS PD gain, White LED, Lo ADC Range       */
    uint32_t lirpd_adchi_irled; /**< Large IR PD gain, IR LED, Hi ADC Range     */
    uint32_t ledi_65ma;         /**< LED Current Ratio at 65 mA                 */
    uint8_t  ucoef[4];          /**< UV Coefficient Storage                     */
};

/***************************************************************************//**
 * @brief
 *   Factory Calibration Reference Values
 ******************************************************************************/
struct cal_ref_t calref = {

    FLT_TO_FX20( 4.021290),  // sirpd_adchi_irled
    FLT_TO_FX20(57.528500),  // sirpd_adclo_irled
    FLT_TO_FX20( 2.690010),  // sirpd_adclo_whled
    FLT_TO_FX20( 0.042903),  // vispd_adchi_whled
    FLT_TO_FX20( 0.633435),  // vispd_adclo_whled
    FLT_TO_FX20(23.902900),  // lirpd_adchi_irled
    FLT_TO_FX20(56.889300),  // ledi_65ma
    {0x7B, 0x6B, 0x01, 0x00} // default ucoef

};


/***************************************************************************//**
 * @brief
 *   Returns the calibration ratio to be applied to VIS measurements
 ******************************************************************************/
static uint32_t vispd_correction(uint8_t* buffer)
{

    struct operand_t op;
    uint32_t         result;
    int16_t          index = 0;

    if( index < 0 ) result = FX20_ONE;

    op.op1 = calref.vispd_adclo_whled;
    op.op2 = VISPD_ADCLO_WHLED;
    result = fx20_divide( &op );

    if( result == FX20_BAD_VALUE ) result = FX20_ONE;

    return result;
}

/***************************************************************************//**
 * @brief
 *   Returns the calibration ratio to be applied to IR measurements
 ******************************************************************************/
static uint32_t irpd_correction(uint8_t* buffer)
{
    struct operand_t op;
    uint32_t         result;
    int16_t          index = 0;

    if( index < 0 ) result = FX20_ONE;

    // op.op1 = SIRPD_ADCLO_IRLED_REF; op.op2 = SIRPD_ADCLO_IRLED;
    op.op1 = calref.sirpd_adclo_irled;
    op.op2 = SIRPD_ADCLO_IRLED;
    result = fx20_divide( &op );

    if( result == FX20_BAD_VALUE ) result = FX20_ONE;

    return result;
}

/***************************************************************************//**
 * @brief
 *   Returns the ratio to correlate between x_RANGE=0 and x_RANGE=1
 *   It is typically 14.5, but may have some slight component-to-component
 *   differences.
 ******************************************************************************/
static uint32_t adcrange_ratio(uint8_t* buffer)
{
    struct operand_t op;
    uint32_t         result;

    op.op1 = SIRPD_ADCLO_IRLED  ;
    op.op2 = SIRPD_ADCHI_IRLED  ;
    result = fx20_divide( &op );

    if( result == FX20_BAD_VALUE ) result = FLT_TO_FX20( 14.5 );

    return result;
}

/***************************************************************************//**
 * @brief
 *   Returns the ratio to correlate between measurements made from large PD
 *   to measurements made from small PD.
 ******************************************************************************/
static uint32_t irsize_ratio(uint8_t* buffer)
{
    struct operand_t op;
    uint32_t         result;

    op.op1 = LIRPD_ADCHI_IRLED  ;
    op.op2 = SIRPD_ADCHI_IRLED  ;

    result = fx20_divide( &op );

    if( result == FX20_BAD_VALUE ) result = FLT_TO_FX20(  6.0 );

    return  result;
}

/***************************************************************************//**
 * @brief
 *   Returns the ratio to adjust for differences in IRLED drive strength. Note
 *   that this does not help with LED irradiance variation.
 ******************************************************************************/
static uint32_t ledi_ratio(uint8_t* buffer)
{

    struct operand_t op;
    uint32_t         result;
    int16_t          index;

    index = 0;

    if( index < 0 ) result = FX20_ONE;

    // op.op1 = LED_DRV65_REF; op.op2 = LED_DRV65;
    op.op1 = calref.ledi_65ma;
    op.op2 = LED_DRV65;
    result = fx20_divide( &op );

    if( result == FX20_BAD_VALUE ) result = FX20_ONE;

    return result;
}

bool Adafruit_SI1145::beginDefault(void)
{

    uint8_t id = read8(SI1145_REG_PARTID);
    logInfo("ID: %X", id);
    if (id != 0x45) return false; // look for SI1145
    
    reset();

    /***********************************/
    // enable UVindex measurement coefficients!

    write8(SI1145_REG_UCOEFF0, 0x7B);
    write8(SI1145_REG_UCOEFF1, 0x6B);
    write8(SI1145_REG_UCOEFF2, 0x01);
    write8(SI1145_REG_UCOEFF3, 0x00);

    // enable UV sensor
    writeParam(SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_ENUV |
               SI1145_PARAM_CHLIST_ENALSIR | SI1145_PARAM_CHLIST_ENALSVIS |
               SI1145_PARAM_CHLIST_ENPS1);
    // enable interrupt on every sample
    write8(SI1145_REG_INTCFG, SI1145_REG_INTCFG_INTOE);
    write8(SI1145_REG_IRQEN, SI1145_REG_IRQEN_ALSEVERYSAMPLE);

    /****************************** Prox Sense 1 */

    // program LED current
    write8(SI1145_REG_PSLED21, 0x03); // 20mA for LED 1 only
    writeParam(SI1145_PARAM_PS1ADCMUX, SI1145_PARAM_ADCMUX_LARGEIR);
    // prox sensor #1 uses LED #1
    writeParam(SI1145_PARAM_PSLED12SEL, SI1145_PARAM_PSLED12SEL_PS1LED1);
    // fastest clocks, clock div 1
    writeParam(SI1145_PARAM_PSADCGAIN, 0);
    // take 511 clocks to measure
    writeParam(SI1145_PARAM_PSADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
    // in prox mode, high range
    writeParam(SI1145_PARAM_PSADCMISC, SI1145_PARAM_PSADCMISC_RANGE|
               SI1145_PARAM_PSADCMISC_PSMODE);

    writeParam(SI1145_PARAM_ALSIRADCMUX, SI1145_PARAM_ADCMUX_SMALLIR);
    // fastest clocks, clock div 1
    writeParam(SI1145_PARAM_ALSIRADCGAIN, 0);
    // take 511 clocks to measure
    writeParam(SI1145_PARAM_ALSIRADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
    // in high range mode
    writeParam(SI1145_PARAM_ALSIRADCMISC, SI1145_PARAM_ALSIRADCMISC_RANGE);


    // fastest clocks, clock div 1
    writeParam(SI1145_PARAM_ALSVISADCGAIN, 0);
    // take 511 clocks to measure
    writeParam(SI1145_PARAM_ALSVISADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
    // in high range mode (not normal signal)
    writeParam(SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_VISRANGE);

    // measurement rate for auto
    write8(SI1145_REG_MEASRATE0, 0xFF); // 255 * 31.25uS = 8ms
    // auto run
    write8(SI1145_REG_COMMAND, SI1145_PSALS_AUTO);
    
    wait(0.05);

    return true;
}

void Adafruit_SI1145::reset()
{
    write8(SI1145_REG_MEASRATE0, 0);
    write8(SI1145_REG_MEASRATE1, 0);
    write8(SI1145_REG_IRQEN, 0);
    write8(SI1145_REG_IRQMODE1, 0);
    write8(SI1145_REG_IRQMODE2, 0);
    write8(SI1145_REG_INTCFG, 0);
    write8(SI1145_REG_IRQSTAT, 0xFF);

    write8(SI1145_REG_COMMAND, SI1145_RESET);
    wait(0.01);
    write8(SI1145_REG_HWKEY, 0x17);

    wait(0.01);
}


/***************************************************************************//**
 * @brief
 *   Initializes the Si113x/46/47/48 UCOEF Registers.
 *
 * @details
 *   Takes in an optional input ucoef pointer, then modifies
 *   it based on calibration. If the input ucoef is NULL, default values for
 *   clear overlay is assumed and then the si114x_cal is used to adjust it
 *   before setting the ucoef. Note that the Si114x ucoef registers are
 *   updated by this function; no additional action is required. This routine
 *   also performs the necessary querying of the chip identification to make
 *   sure that it is a uvindex-capable device.
 * @param[in] si114x_handle
 *   The programmer's toolkit handle
 * @param[in] input_ucoef
 *   if NULL, a clear overlay is assumed, and datasheet values for ucoef is
 *   used. Otherwise, pointer to 4 bytes array representing the reference
 *   coefficients is passed.
 * @param[in] si114x_cal
 *   Points to the SI114X_CAL_S structure that holds the calibration values
 *   from the Si113x/4x
 * @retval   0
 *   Success
 * @retval  -1
 *   The device is neither Si1132, Si1145, Si1146 nor Si1147
 * @retval  <-1
 *   Error
 ******************************************************************************/
void Adafruit_SI1145::si114x_set_ucoef( uint8_t*      input_ucoef,
                          SI114X_CAL_S* si114x_cal )
{


    uint32_t         vc=FX20_ONE, ic=FX20_ONE, long_temp;
    struct operand_t op;
    uint8_t*         ref_ucoef = si114x_cal->ucoef_p;
    uint8_t          out_ucoef[4];

    if( input_ucoef != NULL ) ref_ucoef = input_ucoef;

    if( ref_ucoef == NULL ){
        while(1);    
    }


    if(si114x_cal != 0) {
        if(si114x_cal->vispd_correction > 0) vc = si114x_cal->vispd_correction;
        if(si114x_cal->irpd_correction  > 0) ic = si114x_cal->irpd_correction;
    }

    op.op1 = ref_ucoef[0] + ((ref_ucoef[1])<<8);
    op.op2 = vc;
    long_temp   = fx20_multiply( &op );
    out_ucoef[0] = (long_temp & 0x00ff);
    out_ucoef[1] = (long_temp & 0xff00)>>8;

    op.op1 = ref_ucoef[2] + (ref_ucoef[3]<<8);
    op.op2 = ic;
    long_temp   = fx20_multiply( &op );
    out_ucoef[2] = (long_temp & 0x00ff);
    out_ucoef[3] = (long_temp & 0xff00)>>8;

    write32(SI1145_REG_UCOEFF0, (char*)out_ucoef);


}


// returns the UV index * 100 (divide by 100 to get the index)
uint16_t Adafruit_SI1145::readUV(void)
{
    return read16(0x2C);
}

// returns visible+IR light levels
uint16_t Adafruit_SI1145::readVisible(void)
{
    return read16(0x22);
}

// returns IR light levels
uint16_t Adafruit_SI1145::readIR(void)
{
    return read16(0x24);
}

// returns "Proximity" - assumes an IR LED is attached to LED
uint16_t Adafruit_SI1145::readProx(void)
{
    return read16(0x26);
}

/*********************************************************************/

uint8_t Adafruit_SI1145::writeParam(uint8_t p, uint8_t v)
{
    //Serial.print("Param 0x"); Serial.print(p, HEX);
    //Serial.print(" = 0x"); Serial.println(v, HEX);

    write8(SI1145_REG_PARAMWR, v);
    write8(SI1145_REG_COMMAND, p | SI1145_PARAM_SET);
    return read8(SI1145_REG_PARAMRD);
}

uint8_t Adafruit_SI1145::readParam(uint8_t p)
{
    write8(SI1145_REG_COMMAND, p | SI1145_PARAM_QUERY);
    return read8(SI1145_REG_PARAMRD);
}

/*********************************************************************/

uint8_t  Adafruit_SI1145::read8(uint8_t reg)
{
    char cmd[1];
    char data[1];

    cmd[0] = reg;
    i2c.write(SI1145_ADDR, cmd, 1);
    i2c.read(SI1145_ADDR, data, 1);
    return data[0];

}

uint16_t Adafruit_SI1145::read16(uint8_t a)
{
    char cmd[1], data[2];
    uint16_t val;

    cmd[0] = a;
    i2c.write(SI1145_ADDR, cmd, 1);
    i2c.read(SI1145_ADDR, data, 2);
    val = data[0] | (data[1] << 8);
    return val;
}

void Adafruit_SI1145::readBlock(uint8_t a, char* buffer, uint8_t num_reads)
{
    char cmd[1];

    cmd[0] = a;
    i2c.write(SI1145_ADDR, cmd, 1);
    i2c.read(SI1145_ADDR, buffer, num_reads);

}

void Adafruit_SI1145::write8(uint8_t reg, uint8_t val)
{
    char cmd[2];
    cmd[0] = reg;
    cmd[1] = val;
    i2c.write(SI1145_ADDR, cmd, 2);
}

void Adafruit_SI1145::write32(uint8_t reg, char* buf)
{
    char cmd[5];
    cmd[0] = reg;
    cmd[1] = buf[0];
    cmd[2] = buf[1];
    cmd[3] = buf[2];
    cmd[4] = buf[3];

    i2c.write(SI1145_ADDR, cmd, 5);
}