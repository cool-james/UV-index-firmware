/**********************************************************************
* COPYRIGHT 2016 MULTI-TECH SYSTEMS, INC.
*
* ALL RIGHTS RESERVED BY AND FOR THE EXCLUSIVE BENEFIT OF
* MULTI-TECH SYSTEMS, INC.
*
* MULTI-TECH SYSTEMS, INC. - CONFIDENTIAL AND PROPRIETARY
* INFORMATION AND/OR TRADE SECRET.
*
* NOTICE: ALL CODE, PROGRAM, INFORMATION, SCRIPT, INSTRUCTION,
* DATA, AND COMMENT HEREIN IS AND SHALL REMAIN THE CONFIDENTIAL
* INFORMATION AND PROPERTY OF MULTI-TECH SYSTEMS, INC.
* USE AND DISCLOSURE THEREOF, EXCEPT AS STRICTLY AUTHORIZED IN A
* WRITTEN AGREEMENT SIGNED BY MULTI-TECH SYSTEMS, INC. IS PROHIBITED.
*
***********************************************************************/

#include "ChannelPlan_AS923.h"
#include "limits.h"

using namespace lora;

const uint8_t ChannelPlan_AS923::AS923_RADIO_POWERS[] = { 3, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 18, 19, 20 };
const uint8_t ChannelPlan_AS923::AS923_MAX_PAYLOAD_SIZE[] = { 51, 51, 51, 115, 242, 242, 242, 242, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t ChannelPlan_AS923::AS923_MAX_PAYLOAD_SIZE_REPEATER[] = { 51, 51, 51, 115, 222, 222, 222, 222, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t ChannelPlan_AS923::AS923_MAX_PAYLOAD_SIZE_400[] = { 0, 0, 19, 61, 133, 242, 242, 242, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t ChannelPlan_AS923::AS923_MAX_PAYLOAD_SIZE_REPEATER_400[] = { 0, 0, 19, 61, 133, 222, 222, 222, 0, 0, 0, 0, 0, 0, 0, 0 };

const uint8_t MAX_ERP_VALUES[] = { 8, 10, 12, 13, 14, 16, 18, 20, 21, 24, 26, 27, 29, 30, 33, 36 };

ChannelPlan_AS923::ChannelPlan_AS923()
:
    ChannelPlan(NULL, NULL)
{

}

ChannelPlan_AS923::ChannelPlan_AS923(Settings* settings)
:
    ChannelPlan(NULL, settings)
{

}

ChannelPlan_AS923::ChannelPlan_AS923(SxRadio* radio, Settings* settings)
:
    ChannelPlan(radio, settings)
{

}

ChannelPlan_AS923::~ChannelPlan_AS923() {

}

void ChannelPlan_AS923::Init() {

    _datarates.clear();
    _channels.clear();
    _dutyBands.clear();

    DutyBand band;

    band.Index = 0;
    band.DutyCycle = 0;

    Datarate dr;

    _plan = AS923;
    _planName = "AS923";
    _maxTxPower = 36;
    _minTxPower = 0;

    _minFrequency = 920000000;
    _maxFrequency = 928000000;

    RADIO_POWERS = AS923_RADIO_POWERS;
    MAX_PAYLOAD_SIZE = AS923_MAX_PAYLOAD_SIZE;
    MAX_PAYLOAD_SIZE_REPEATER = AS923_MAX_PAYLOAD_SIZE_REPEATER;

    _minDatarate = 0;
    _maxDatarate = 7;

    _minRx2Datarate = DR_0;
    _maxRx2Datarate = DR_7;

    _minDatarateOffset = 0;
    _maxDatarateOffset = 7;

    _numChans125k = 16;
    _numChans500k = 0;

    GetSettings()->Session.Rx2Frequency = 923200000;
    GetSettings()->Session.Rx2DatarateIndex = DR_2;
    GetSettings()->Session.Max_EIRP  = 20;

    logInfo("Initialize datarates...");

    dr.SpreadingFactor = SF_12;

    // Add DR0-5
    while (dr.SpreadingFactor >= SF_7) {
        AddDatarate(-1, dr);
        dr.SpreadingFactor--;
        dr.Index++;
    }

    // Add DR6
    dr.SpreadingFactor = SF_7;
    dr.Bandwidth = BW_250;
    AddDatarate(-1, dr);
    dr.Index++;

    // Add DR7
    dr.SpreadingFactor = SF_FSK;
    dr.Bandwidth = BW_FSK;
    dr.PreambleLength = 10;
    dr.Coderate = 0;
    AddDatarate(-1, dr);
    dr.Index++;

    _maxDatarate = DR_7;

    // Skip DR8-15 RFU
    dr.SpreadingFactor = SF_INVALID;
    while (dr.Index++ < DR_15) {
        AddDatarate(-1, dr);
    }

    GetSettings()->Session.TxDatarate = 0;

    logInfo("Initialize channels...");

    Channel chan;
    chan.DrRange.Fields.Min = DR_0;
    chan.DrRange.Fields.Max = DR_5;
    chan.Index = 0;
    chan.Frequency = 923200000;
    SetNumberOfChannels(16);

    for (uint8_t i = 0; i < 2; i++) {
        AddChannel(i, chan);
        chan.Index++;
        chan.Frequency += 200000;
    }

    chan.DrRange.Value = 0;
    chan.Frequency = 0;

    for (uint8_t i = 2; i < 16; i++) {
        AddChannel(i, chan);
        chan.Index++;
    }

    SetChannelMask(0, 0x03);


    // Add downlink channel defaults
    chan.Index = 0;
    _dlChannels.resize(16);
    for (uint8_t i = 0; i < 16; i++) {
        AddDownlinkChannel(i, chan);
        chan.Index++;
    }

    band.Index = 0;
    band.FrequencyMin = _minFrequency;
    band.FrequencyMax = _maxFrequency;
    band.PowerMax = 14;
    band.TimeOffEnd = 0;

    // Disable duty-cycle limits
    band.DutyCycle = 0;

    AddDutyBand(-1, band);

    GetSettings()->Session.TxPower = GetSettings()->Network.TxPower;
}

uint8_t ChannelPlan_AS923::AddChannel(int8_t index, Channel channel) {
    logTrace("Add Channel %d : %lu : %02x %d", index, channel.Frequency, channel.DrRange.Value, _channels.size());

    assert(index < (int) _channels.size());

    if (index >= 0) {
        _channels[index] = channel;
    } else {
        _channels.push_back(channel);
    }

    return LORA_OK;
}

uint8_t ChannelPlan_AS923::HandleJoinAccept(const uint8_t* buffer, uint8_t size) {

    if (size == 33) {
        Channel ch;
        int index = 2;
        for (int i = 13; i < size - 5; i += 3) {

            ch.Frequency = ((buffer[i]) | (buffer[i + 1] << 8) | (buffer[i + 2] << 16)) * 100u;

            if (ch.Frequency > 0) {
                ch.Index = index;
                ch.DrRange.Fields.Min = static_cast<int8_t>(DR_0);
                ch.DrRange.Fields.Max = static_cast<int8_t>(DR_5);
                AddChannel(index, ch);

                if (GetDutyBand(ch.Frequency) > -1)
                    _channelMask[0] |= (1 << index);
                else
                    _channelMask[0] |= ~(1 << index);

                index += 1;
            }
        }
    }

    return LORA_OK;
}

uint8_t ChannelPlan_AS923::SetTxConfig() {

    logInfo("Configure radio for TX");

    Datarate txDr = GetDatarate(GetSettings()->Session.TxDatarate);
    int8_t max_pwr = GetSettings()->Session.Max_EIRP;

    int8_t pwr = 0;

    pwr = std::min < int8_t > (GetSettings()->Session.TxPower, (max_pwr - GetSettings()->Network.AntennaGain));

    for (int i = 20; i >= 0; i--) {
        if (RADIO_POWERS[i] <= pwr) {
            pwr = i;
            break;
        }
        if (i == 0) {
            pwr = i;
        }
    }

    logDebug("Session pwr: %d ant: %d max: %d", GetSettings()->Session.TxPower, GetSettings()->Network.AntennaGain, max_pwr);
    logDebug("Radio Power index: %d output: %d total: %d", pwr, RADIO_POWERS[pwr], RADIO_POWERS[pwr] + GetSettings()->Network.AntennaGain);

    uint32_t bw = txDr.Bandwidth;
    uint32_t sf = txDr.SpreadingFactor;
    uint8_t cr = txDr.Coderate;
    uint8_t pl = txDr.PreambleLength;
    uint16_t fdev = 0;
    bool crc = txDr.Crc;
    bool iq = txDr.TxIQ;

    if (GetSettings()->Network.DisableCRC == true)
        crc = false;

    SxRadio::RadioModems_t modem = SxRadio::MODEM_LORA;

    if (sf == SF_FSK) {
        modem = SxRadio::MODEM_FSK;
        sf = 50e3;
        fdev = 25e3;
        bw = 0;
    }

    GetRadio()->SetTxConfig(modem, pwr, fdev, bw, sf, cr, pl, false, crc, false, 0, iq, 3e3);

    logDebug("TX PWR: %u DR: %u SF: %u BW: %u CR: %u PL: %u CRC: %d IQ: %d", pwr, txDr.Index, sf, bw, cr, pl, crc, iq);

    return LORA_OK;
}

uint8_t ChannelPlan_AS923::SetRxConfig(uint8_t window, bool continuous) {

    RxWindow rxw = GetRxWindow(window);

    if (_dlChannels[_txChannel].Frequency != 0)
        GetRadio()->SetChannel(_dlChannels[_txChannel].Frequency);
    else
        GetRadio()->SetChannel(rxw.Frequency);

    Datarate rxDr = GetDatarate(rxw.DatarateIndex);
    uint32_t bw = rxDr.Bandwidth;
    uint32_t sf = rxDr.SpreadingFactor;
    uint8_t cr = rxDr.Coderate;
    uint8_t pl = rxDr.PreambleLength;
    uint16_t sto = rxDr.SymbolTimeout();
    uint32_t afc = 0;
    bool crc = false; // downlink does not use CRC according to LORAWAN

    if (GetSettings()->Network.DisableCRC == true)
        crc = false;

    Datarate txDr = GetDatarate(GetSettings()->Session.TxDatarate);
    bool iq = txDr.RxIQ;

    if (P2PEnabled()) {
        iq = txDr.TxIQ;
    }

    SxRadio::RadioModems_t modem = SxRadio::MODEM_LORA;

    if (sf == SF_FSK) {
        modem = SxRadio::MODEM_FSK;
        sf = 50e3;
        cr = 0;
        bw = 50e3;
        afc = 83333;
        iq = false;
        crc = true;  // FSK must use CRC
    }

    // Disable printf's to actually receive packets, printing to debug may mess up the timing
    // logTrace("Configure radio for RX%d on freq: %lu", window, rxw.Frequency);
    // logTrace("RX SF: %u BW: %u CR: %u PL: %u STO: %u CRC: %d IQ: %d", sf, bw, cr, pl, sto, crc, iq);

    GetRadio()->SetRxConfig(modem, bw, sf, cr, afc, pl, sto, false, 0, crc, false, 0, iq, continuous);

    return LORA_OK;
}

Channel ChannelPlan_AS923::GetChannel(int8_t index) {
    Channel chan;
    memset(&chan, 0, sizeof(Channel));

    chan = _channels[index];

    return chan;
}

uint8_t ChannelPlan_AS923::SetFrequencySubBand(uint8_t sub_band) {
    return LORA_OK;
}

void ChannelPlan_AS923::LogRxWindow(uint8_t wnd) {

    RxWindow rxw = GetRxWindow(wnd);
    Datarate rxDr = GetDatarate(rxw.DatarateIndex);
    uint8_t bw = rxDr.Bandwidth;
    uint8_t sf = rxDr.SpreadingFactor;
    uint8_t cr = rxDr.Coderate;
    uint8_t pl = rxDr.PreambleLength;
    uint16_t sto = rxDr.SymbolTimeout();
    bool crc = false; // downlink does not use CRC according to LORAWAN
    bool iq = GetTxDatarate().RxIQ;
    uint32_t freq = rxw.Frequency;

    if (_dlChannels[_txChannel].Frequency != 0)
        freq = _dlChannels[_txChannel].Frequency;

    logTrace("RX%d on freq: %lu", wnd, freq);
    logTrace("RX DR: %u SF: %u BW: %u CR: %u PL: %u STO: %u CRC: %d IQ: %d", rxDr.Index, sf, bw, cr, pl, sto, crc, iq);
}

uint8_t ChannelPlan_AS923::GetMaxPayloadSize() {
    if (GetSettings()->Session.UplinkDwelltime == 1) {
        if (GetSettings()->Network.RepeaterMode)
            return AS923_MAX_PAYLOAD_SIZE_REPEATER_400[GetSettings()->Session.TxDatarate];
        else
            return AS923_MAX_PAYLOAD_SIZE_400[GetSettings()->Session.TxDatarate];
    } else {
        if (GetSettings()->Network.RepeaterMode)
            return MAX_PAYLOAD_SIZE_REPEATER[GetSettings()->Session.TxDatarate];
        else
            return MAX_PAYLOAD_SIZE[GetSettings()->Session.TxDatarate];
    }
}

RxWindow ChannelPlan_AS923::GetRxWindow(uint8_t window) {
    RxWindow rxw;
    int index = 0;

    if (P2PEnabled()) {
        rxw.Frequency = GetSettings()->Network.TxFrequency;
        index = GetSettings()->Session.TxDatarate;
    } else {
        if (window == 1) {
            // Use same frequency as TX
            rxw.Frequency = _channels[_txChannel].Frequency;

            if (GetSettings()->Session.Rx1DatarateOffset >= 6) {
                index =  GetSettings()->Session.TxDatarate + (GetSettings()->Session.Rx1DatarateOffset == 6 ? 1 : 2);
                index = std::min<int>(index, _maxDatarate);
            } else if (GetSettings()->Session.TxDatarate > GetSettings()->Session.Rx1DatarateOffset) {
                index = GetSettings()->Session.TxDatarate - GetSettings()->Session.Rx1DatarateOffset;
            } else {
                index = 0;
            }

            // LoRaWAN 1.0.2 Sec 2.7.7
            uint8_t minDr = GetSettings()->Session.DownlinkDwelltime == 1 ? 2 : 0;
            index = std::min<uint8_t>(5, std::max<uint8_t>(minDr, index));

        } else {
            // Use session RX2 frequency
            rxw.Frequency = GetSettings()->Session.Rx2Frequency;
            index = GetSettings()->Session.Rx2DatarateIndex;
        }
    }

    rxw.DatarateIndex = index;

    return rxw;
}

uint8_t ChannelPlan_AS923::HandleRxParamSetup(const uint8_t* payload, uint8_t index, uint8_t size, uint8_t& status) {
    status = 0x07;
    int8_t datarate = 0;
    int8_t drOffset = 0;
    uint32_t freq = 0;

    drOffset = payload[index++];
    datarate = drOffset & 0x0F;
    drOffset = (drOffset >> 4) & 0x07;

    freq = payload[index++];
    freq |= payload[index++] << 8;
    freq |= payload[index++] << 16;
    freq *= 100;

    if (!CheckRfFrequency(freq)) {
        logInfo("Freq KO");
        status &= 0xFE; // Channel frequency KO
    }

    if (datarate < _minRx2Datarate || datarate > _maxRx2Datarate) {
        logInfo("DR KO");
        status &= 0xFD; // Datarate KO
    }

    if (drOffset < 0 || drOffset > _maxDatarateOffset) {
        logInfo("DR Offset KO");
        status &= 0xFB; // Rx1DrOffset range KO
    }

    if ((status & 0x07) == 0x07) {
        logInfo("RxParamSetup accepted Rx2DR: %d Rx2Freq: %d Rx1Offset: %d", datarate, freq, drOffset);
        SetRx2DatarateIndex(datarate);
        SetRx2Frequency(freq);
        SetRx1Offset(drOffset);
    } else {
        logInfo("RxParamSetup rejected Rx2DR: %d Rx2Freq: %d Rx1Offset: %d", datarate, freq, drOffset);
    }

    return LORA_OK;
}

uint8_t ChannelPlan_AS923::HandleNewChannel(const uint8_t* payload, uint8_t index, uint8_t size, uint8_t& status) {

    status = 0x03;
    uint8_t channelIndex = 0;
    Channel chParam;

    channelIndex = payload[index++];
    lora::CopyFreqtoInt(payload + index, chParam.Frequency);
    index += 3;
    chParam.DrRange.Value = payload[index++];

    if (channelIndex < 2 || channelIndex > _channels.size() - 1) {
        logError("New Channel index KO");
        status &= 0xFE; // Channel index KO
    }

    if (!GetRadio()->CheckRfFrequency(chParam.Frequency)) {
        logError("New Channel frequency KO");
        status &= 0xFE; // Channel frequency KO
    }

    if (chParam.DrRange.Fields.Min > chParam.DrRange.Fields.Max) {
        logError("New Channel datarate min/max KO");
        status &= 0xFD; // Datarate range KO
    } else if (chParam.DrRange.Fields.Min < _minDatarate || chParam.DrRange.Fields.Min > _maxDatarate) {
        logError("New Channel datarate min KO");
        status &= 0xFD; // Datarate range KO
    } else if (chParam.DrRange.Fields.Max < _minDatarate || chParam.DrRange.Fields.Max > _maxDatarate) {
        logError("New Channel datarate max KO");
        status &= 0xFD; // Datarate range KO
    }

    if ((status & 0x03) == 0x03) {
        logInfo("New Channel accepted index: %d freq: %lu drRange: %02x", channelIndex, chParam.Frequency, chParam.DrRange.Value);
        AddChannel(channelIndex, chParam);
        SetChannelMask(0, _channelMask[0] | 1 << (channelIndex));
    }

    return LORA_OK;
}

uint8_t ChannelPlan_AS923::HandlePingSlotChannelReq(const uint8_t* payload, uint8_t index, uint8_t size, uint8_t& status) {

    lora::CopyFreqtoInt(payload + index, _beaconRxChannel.Frequency);
    index += 3;

    if (_beaconRxChannel.Frequency != 0) {
        _beaconRxChannel.DrRange.Value = payload[index];
    } else {
        // TODO: set to default beacon rx channel
    }

    status = 0x03;
    return LORA_OK;
}

uint8_t ChannelPlan_AS923::HandleBeaconFrequencyReq(const uint8_t* payload, uint8_t index, uint8_t size, uint8_t& status) {

    status = 0x03;
    Channel chParam;

    // Skip channel index
    index++;

    lora::CopyFreqtoInt(payload + index, chParam.Frequency);
    index += 3;
    chParam.DrRange.Value = payload[index++];

    if (!GetRadio()->CheckRfFrequency(chParam.Frequency)) {
        status &= 0xFE; // Channel frequency KO
    }

    if (chParam.DrRange.Fields.Min < chParam.DrRange.Fields.Max) {
        status &= 0xFD; // Datarate range KO
    } else if (chParam.DrRange.Fields.Min < _minDatarate || chParam.DrRange.Fields.Min > _maxDatarate) {
        status &= 0xFD; // Datarate range KO
    } else if (chParam.DrRange.Fields.Max < _minDatarate || chParam.DrRange.Fields.Max > _maxDatarate) {
        status &= 0xFD; // Datarate range KO
    }

    if ((status & 0x03) == 0x03) {
        _beaconChannel = chParam;
    }

    if (_beaconChannel.Frequency == 0) {
        // TODO: Set to default
    }

    status = 0x01;

    return LORA_OK;
}

uint8_t ChannelPlan_AS923::HandleAdrCommand(const uint8_t* payload, uint8_t index, uint8_t size, uint8_t& status) {

    uint8_t power = 0;
    uint8_t datarate = 0;
    uint16_t mask = 0;
    uint16_t new_mask = 0;
    uint8_t ctrl = 0;
    uint8_t nbRep = 0;

    status = 0x07;
    datarate = payload[index++];
    power = datarate & 0x0F;
    datarate = (datarate >> 4) & 0x0F;

    mask = payload[index++];
    mask |= payload[index++] << 8;

    nbRep = payload[index++];
    ctrl = (nbRep >> 4) & 0x07;
    nbRep &= 0x0F;

    if (nbRep == 0) {
        nbRep = 1;
    }

    if (datarate > _maxDatarate) {
        status &= 0xFD; // Datarate KO
    }
    //
    // Remark MaxTxPower = 0 and MinTxPower = 7
    //
    if (power < 0 || power > 7) {
        status &= 0xFB; // TxPower KO
    }

    switch (ctrl) {
        case 0:
            SetChannelMask(0, mask);
            break;

        case 6:
            // enable all currently defined channels
            // set bits 0 - N of a number by (2<<N)-1
            new_mask = (1 << _channels.size()) - 1;
            SetChannelMask(0, new_mask);
            break;

        default:
            logWarning("rejecting RFU or unknown control value %d", ctrl);
            status &= 0xFE; // ChannelMask KO
            return LORA_ERROR;
    }

    if (GetSettings()->Network.ADREnabled) {
        GetSettings()->Session.TxDatarate = datarate;
        GetSettings()->Session.TxPower = GetSettings()->Session.Max_EIRP - (power * 2);
        GetSettings()->Session.Redundancy = nbRep;
    } else {
        logDebug("ADR is disabled, DR and Power not changed.");
        status &= 0xFB; // TxPower KO
        status &= 0xFD; // Datarate KO
    }

    logDebug("ADR DR: %u PWR: %u Ctrl: %02x Mask: %04x NbRep: %u Stat: %02x", datarate, power, ctrl, mask, nbRep, status);

    return LORA_OK;
}

uint8_t ChannelPlan_AS923::ValidateAdrConfiguration() {
    uint8_t status = 0x07;
    uint8_t datarate = GetSettings()->Session.TxDatarate;
    uint8_t power = GetSettings()->Session.TxPower;

    if (!GetSettings()->Network.ADREnabled) {
        logDebug("ADR disabled - no applied changes to validate");
        return status;
    }

    if (datarate > _maxDatarate) {
        logWarning("ADR Datarate KO - outside allowed range");
        status &= 0xFD; // Datarate KO
    }
    if (power < _minTxPower || power > _maxTxPower) {
        logWarning("ADR TX Power KO - outside allowed range");
        status &= 0xFB; // TxPower KO
    }

    // default channels must be enabled
    if ((_channelMask[0] & 0x0003) != 0x0003) {
        logWarning("ADR Channel Mask KO - default channels must be enabled");
        status &= 0xFE; // ChannelMask KO
    }

    // mask must not contain any undefined channels
    for (int i = 2; i < 16; i++) {
        if ((_channelMask[0] & (1 << i)) && (_channels[i].Frequency == 0)) {
            logWarning("ADR Channel Mask KO - cannot enable undefined channel");
            status &= 0xFE; // ChannelMask KO
            break;
        }
    }

    return status;
}

uint8_t ChannelPlan_AS923::HandleAckTimeout() {

    if (!GetSettings()->Network.ADREnabled) {
        return LORA_ADR_OFF;
    }

    if ((++(GetSettings()->Session.AckCounter) % 2) == 0) {
        if (GetSettings()->Session.TxPower < GetSettings()->Network.TxPowerMax) {
            logTrace("ADR Setting power to maximum");
            GetSettings()->Session.TxPower = GetSettings()->Network.TxPowerMax;
        } else if (GetSettings()->Session.TxDatarate > 0) {
            logTrace("ADR Lowering datarate");
            GetSettings()->Session.TxDatarate--;
        }
    }

    return LORA_OK;
}


uint32_t ChannelPlan_AS923::GetTimeOffAir()
{
    if (GetSettings()->Test.DisableDutyCycle == lora::ON)
        return 0;

    uint32_t min = 0;
    uint32_t now = _dutyCycleTimer.read_ms();


    min = UINT_MAX;
    int8_t band = 0;

    if (P2PEnabled()) {
        int8_t band = GetDutyBand(GetSettings()->Network.TxFrequency);
        if (_dutyBands[band].TimeOffEnd > now) {
            min = _dutyBands[band].TimeOffEnd - now;
        } else {
            min = 0;
        }
    } else {
        for (size_t i = 0; i < _channels.size(); i++) {
            if (IsChannelEnabled(i) && GetChannel(i).Frequency != 0 &&
                !(GetSettings()->Session.TxDatarate < GetChannel(i).DrRange.Fields.Min ||
                  GetSettings()->Session.TxDatarate > GetChannel(i).DrRange.Fields.Max)) {

                band = GetDutyBand(GetChannel(i).Frequency);
                if (band != -1) {
                    // logDebug("band: %d time-off: %d now: %d", band, _dutyBands[band].TimeOffEnd, now);
                    if (_dutyBands[band].TimeOffEnd > now) {
                        min = std::min < uint32_t > (min, _dutyBands[band].TimeOffEnd - now);
                    } else {
                        min = 0;
                        break;
                    }
                }
            }
        }
    }


    if (GetSettings()->Session.AggregatedTimeOffEnd > 0 && GetSettings()->Session.AggregatedTimeOffEnd > now) {
        min = std::max < uint32_t > (min, GetSettings()->Session.AggregatedTimeOffEnd - now);
    }

    now = time(NULL);
    uint32_t join_time = 0;

    if (GetSettings()->Session.JoinFirstAttempt != 0 && now < GetSettings()->Session.JoinTimeOffEnd) {
        join_time = (GetSettings()->Session.JoinTimeOffEnd - now) * 1000;
    }

    min = std::max < uint32_t > (join_time, min);

    return min;
}


void ChannelPlan_AS923::UpdateDutyCycle(uint32_t freq, uint32_t time_on_air_ms) {
    if (GetSettings()->Test.DisableDutyCycle == lora::ON) {
        _dutyCycleTimer.stop();
        for (size_t i = 0; i < _dutyBands.size(); i++) {
            _dutyBands[i].TimeOffEnd = 0;
        }
        return;
    }

    _dutyCycleTimer.start();

    if (GetSettings()->Session.MaxDutyCycle > 0 && GetSettings()->Session.MaxDutyCycle <= 15) {
        GetSettings()->Session.AggregatedTimeOffEnd = _dutyCycleTimer.read_ms() + time_on_air_ms * GetSettings()->Session.AggregateDutyCycle;
        logDebug("Updated Aggregate DCycle Time-off: %lu DC: %f%%", GetSettings()->Session.AggregatedTimeOffEnd, 1 / float(GetSettings()->Session.AggregateDutyCycle));
    } else {
        GetSettings()->Session.AggregatedTimeOffEnd = 0;
    }


    uint32_t time_off_air = 0;
    uint32_t now = _dutyCycleTimer.read_ms();

    for (size_t i = 0; i < _dutyBands.size(); i++) {
        if (_dutyBands[i].TimeOffEnd < now) {
            _dutyBands[i].TimeOffEnd = 0;
        } else {
            _dutyBands[i].TimeOffEnd -= now;
        }

        if (freq >= _dutyBands[i].FrequencyMin && freq <= _dutyBands[i].FrequencyMax) {
            logDebug("update TOE: freq: %d i:%d toa: %d DC:%d", freq, i, time_on_air_ms, _dutyBands[i].DutyCycle);

            if (freq > _minFrequency && freq < _maxFrequency && (GetSettings()->Session.TxPower + GetSettings()->Network.AntennaGain) <= 7) {
                _dutyBands[i].TimeOffEnd = 0;
            } else {
                time_off_air = time_on_air_ms * _dutyBands[i].DutyCycle;
                _dutyBands[i].TimeOffEnd = time_off_air;
            }
        }
    }


    ResetDutyCycleTimer();
}

std::vector<uint32_t> lora::ChannelPlan_AS923::GetChannels() {
    std::vector < uint32_t > chans;

    for (int8_t i = 0; i < (int) _channels.size(); i++) {
        chans.push_back(_channels[i].Frequency);
    }
    chans.push_back(GetRxWindow(2).Frequency);

    return chans;
}

std::vector<uint8_t> lora::ChannelPlan_AS923::GetChannelRanges() {
    std::vector < uint8_t > ranges;

    for (int8_t i = 0; i < (int) _channels.size(); i++) {
        ranges.push_back(_channels[i].DrRange.Value);
    }

    ranges.push_back(GetRxWindow(2).DatarateIndex);

    return ranges;

}

void lora::ChannelPlan_AS923::EnableDefaultChannels() {
    _channelMask[0] |= 0x0003;
}

uint8_t ChannelPlan_AS923::GetNextChannel()
{
    if (GetSettings()->Session.AggregatedTimeOffEnd != 0) {
        return LORA_AGGREGATED_DUTY_CYCLE;
    }

    if (P2PEnabled() || GetSettings()->Network.TxFrequency != 0) {
        logDebug("Using frequency %d", GetSettings()->Network.TxFrequency);

        if (GetSettings()->Test.DisableDutyCycle != lora::ON) {
            int8_t band = GetDutyBand(GetSettings()->Network.TxFrequency);
            logDebug("band: %d freq: %d", band, GetSettings()->Network.TxFrequency);
            if (band != -1 && _dutyBands[band].TimeOffEnd != 0) {
                return LORA_NO_CHANS_ENABLED;
            }
        }

        GetRadio()->SetChannel(GetSettings()->Network.TxFrequency);
        return LORA_OK;
    }

    uint8_t start = 0;
    uint8_t maxChannels = _numChans125k;
    uint8_t nbEnabledChannels = 0;
    uint8_t *enabledChannels = new uint8_t[maxChannels];

    if (GetTxDatarate().Bandwidth == BW_500) {
        maxChannels = _numChans500k;
        start = _numChans125k;
    }

// Search how many channels are enabled
    DatarateRange range;
    uint8_t dr_index = GetSettings()->Session.TxDatarate;
    uint32_t now = _dutyCycleTimer.read_ms();

    for (size_t i = 0; i < _dutyBands.size(); i++) {
        if (_dutyBands[i].TimeOffEnd < now || GetSettings()->Test.DisableDutyCycle == lora::ON) {
            _dutyBands[i].TimeOffEnd = 0;
        }
    }

    for (uint8_t i = start; i < start + maxChannels; i++) {
        range = GetChannel(i).DrRange;
        // logDebug("chan: %d freq: %d range:%02x", i, GetChannel(i).Frequency, range.Value);

        if (IsChannelEnabled(i) && (dr_index >= range.Fields.Min && dr_index <= range.Fields.Max)) {
            int8_t band = GetDutyBand(GetChannel(i).Frequency);
            // logDebug("band: %d freq: %d", band, _channels[i].Frequency);
            if (band != -1 && _dutyBands[band].TimeOffEnd == 0) {
                enabledChannels[nbEnabledChannels++] = i;
            }
        }
    }

    logTrace("Number of available channels: %d", nbEnabledChannels);

    uint32_t freq = 0;
    uint8_t sf = GetTxDatarate().SpreadingFactor;
    uint8_t bw = GetTxDatarate().Bandwidth;
    int16_t thres = DEFAULT_FREE_CHAN_RSSI_THRESHOLD;

    if (nbEnabledChannels == 0) {
        delete [] enabledChannels;
        return LORA_NO_CHANS_ENABLED;
    }

    if (GetSettings()->Network.CADEnabled) {
        // Search for free channel with ms timeout
        int16_t timeout = 10000;
        Timer tmr;
        tmr.start();

        for (uint8_t j = rand_r(0, nbEnabledChannels - 1); tmr.read_ms() < timeout; j++) {
            freq = GetChannel(enabledChannels[j]).Frequency;

            // Listen before talk
            if (GetRadio()->IsChannelFree(SxRadio::MODEM_LORA, freq, sf, thres, bw)) {
                _txChannel = enabledChannels[j];
                break;
            }
        }
    } else {
        uint8_t j = rand_r(0, nbEnabledChannels - 1);
        _txChannel = enabledChannels[j];
        freq = GetChannel(_txChannel).Frequency;
    }

    assert(freq != 0);

    logDebug("Using channel %d : %d", _txChannel, freq);
    GetRadio()->SetChannel(freq);

    delete [] enabledChannels;
    return LORA_OK;
}


uint8_t lora::ChannelPlan_AS923::GetJoinDatarate() {
    uint8_t dr = GetSettings()->Session.TxDatarate;
    
    // Default join datarate is DR2:SF10BW125
    dr = lora::DR_2;
    
    return dr;
}

uint8_t ChannelPlan_AS923::CalculateJoinBackoff(uint8_t size) {

    time_t now = time(NULL);
    uint32_t time_on_max = 0;
    static uint32_t time_off_max = 15;
    uint32_t rand_time_off = 0;

    // TODO: calc time-off-max based on RTC time from JoinFirstAttempt, time-off-max is lost over sleep

    if ((time_t)GetSettings()->Session.JoinTimeOffEnd > now) {
        return LORA_JOIN_BACKOFF;
    }

    uint32_t secs_since_first_attempt = (now - GetSettings()->Session.JoinFirstAttempt);
    uint16_t hours_since_first_attempt = secs_since_first_attempt / (60 * 60);

    static uint8_t join_cnt = 0;

    join_cnt = (join_cnt+1) % 8;

    if (GetSettings()->Session.JoinFirstAttempt == 0) {
        /* 1 % duty-cycle for first hour
         * 0.1 % next 10 hours
         * 0.01 % upto 24 hours         */
        GetSettings()->Session.JoinFirstAttempt = now;
        GetSettings()->Session.JoinTimeOnAir += GetTimeOnAir(size);
        GetSettings()->Session.JoinTimeOffEnd = now + (GetTimeOnAir(size) / 10);
    } else if (join_cnt == 0) {
        if (hours_since_first_attempt < 1) {
            time_on_max = 36000;
            rand_time_off = rand_r(time_off_max - 1, time_off_max + 1);
            // time off max 1 hour
            time_off_max = std::min < uint32_t > (time_off_max * 2, 60 * 60);

            if (GetSettings()->Session.JoinTimeOnAir < time_on_max) {
                GetSettings()->Session.JoinTimeOnAir += GetTimeOnAir(size);
                GetSettings()->Session.JoinTimeOffEnd = now + rand_time_off;
            } else {
                logWarning("Max time-on-air limit met for current join backoff period");
                GetSettings()->Session.JoinTimeOffEnd = GetSettings()->Session.JoinFirstAttempt + 60 * 60;
            }
        } else if (hours_since_first_attempt < 11) {
            if (GetSettings()->Session.JoinTimeOnAir < 36000) {
                GetSettings()->Session.JoinTimeOnAir = 36000;
            }
            time_on_max = 72000;
            rand_time_off = rand_r(time_off_max - 1, time_off_max + 1);
            // time off max 1 hour
            time_off_max = std::min < uint32_t > (time_off_max * 2, 60 * 60);

            if (GetSettings()->Session.JoinTimeOnAir < time_on_max) {
                GetSettings()->Session.JoinTimeOnAir += GetTimeOnAir(size);
                GetSettings()->Session.JoinTimeOffEnd = now + rand_time_off;
            } else {
                logWarning("Max time-on-air limit met for current join backoff period");
                GetSettings()->Session.JoinTimeOffEnd = GetSettings()->Session.JoinFirstAttempt + 11 * 60 * 60;
            }
        } else {
            if (GetSettings()->Session.JoinTimeOnAir < 72000) {
                GetSettings()->Session.JoinTimeOnAir = 72000;
            }
            uint32_t join_time = 2500;

            time_on_max = 80700;
            time_off_max = 1 * 60 * 60; // 1 hour
            rand_time_off = rand_r(time_off_max - 1, time_off_max + 1);

            if (GetSettings()->Session.JoinTimeOnAir < time_on_max - join_time) {
                GetSettings()->Session.JoinTimeOnAir += GetTimeOnAir(size);
                GetSettings()->Session.JoinTimeOffEnd = now + rand_time_off;
            } else {
                logWarning("Max time-on-air limit met for current join backoff period");
                // Reset the join time on air and set end of restriction to the next 24 hour period
                GetSettings()->Session.JoinTimeOnAir = 72000;
                uint16_t days = (now - GetSettings()->Session.JoinFirstAttempt) / (24 * 60 * 60) + 1;
                logWarning("days : %d", days);
                GetSettings()->Session.JoinTimeOffEnd = GetSettings()->Session.JoinFirstAttempt + ((days * 24) + 11) * 60 * 60;
            }
        }

        logWarning("JoinBackoff: %lu seconds  Time On Air: %lu / %lu", GetSettings()->Session.JoinTimeOffEnd - now, GetSettings()->Session.JoinTimeOnAir, time_on_max);
    } else {
        GetSettings()->Session.JoinTimeOnAir += GetTimeOnAir(size);
        GetSettings()->Session.JoinTimeOffEnd = now + (GetTimeOnAir(size) / 10);
    }

    return LORA_OK;
}


uint8_t ChannelPlan_AS923::HandleMacCommand(uint8_t* payload, uint8_t& index) {
    logDebug("AS923 Handle Mac index: %d", index);

    switch (payload[index++]) {
        case SRV_MAC_TX_PARAM_SETUP_REQ: {
            uint8_t eirp_dwell = payload[index++];

            GetSettings()->Session.DownlinkDwelltime = eirp_dwell >> 5 & 0x01;
            GetSettings()->Session.UplinkDwelltime = eirp_dwell >> 4 & 0x01;
            GetSettings()->Session.Max_EIRP = MAX_ERP_VALUES[(eirp_dwell & 0x0F)];
            logDebug("buffer index %d", GetSettings()->Session.CommandBufferIndex);
            if (GetSettings()->Session.CommandBufferIndex < COMMANDS_BUFFER_SIZE) {
                logDebug("Add tx param setup mac cmd to buffer");
                GetSettings()->Session.CommandBuffer[GetSettings()->Session.CommandBufferIndex++] = MOTE_MAC_TX_PARAM_SETUP_ANS;
            }

            logDebug("TX PARAM DWELL UL: %d DL: %d Max EIRP: %d", GetSettings()->Session.UplinkDwelltime, GetSettings()->Session.DownlinkDwelltime, GetSettings()->Session.Max_EIRP);
            break;
        }
        case SRV_MAC_DL_CHANNEL_REQ: {
            uint8_t status = 0x03;
            uint8_t channelIndex = 0;
            Channel chParam;

            channelIndex = payload[index++];
            lora::CopyFreqtoInt(payload + index, chParam.Frequency);
            index += 3;

            chParam.Index = channelIndex;
            chParam.DrRange.Value = 0;

            if (channelIndex > 15) {
                status = 0x00;
            } else if (_channels[channelIndex].Frequency == 0) {
                status &= 0x02;
            } else if (chParam.Frequency != 0 && (chParam.Frequency < _minFrequency || chParam.Frequency > _maxFrequency)) {
                status &= 0x01;
            }

            if (status == 0x03 && GetSettings()->Session.CommandBufferIndex+1 < COMMANDS_BUFFER_SIZE) {
                AddDownlinkChannel(channelIndex, chParam);
            }

            GetSettings()->Session.DlChannelReqAnswer = status;

            logDebug("DL Channel: index: %d freq: %d status: %d", channelIndex, chParam.Frequency, status);
            break;
        }
        default: {
            return LORA_ERROR;
        }
    }

    return LORA_OK;
}
