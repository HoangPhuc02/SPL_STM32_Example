/*
 * =============================================================================
 * Project: STM32F103 CAN Module Implementation
 * File: can_module.c
 * Author: hoangphuc540202@gmail.com
 * Date: August 2025
 * 
 * Description: Implementation of comprehensive CAN module with Data Frame, 
 *              Remote Frame, and Error Frame handling
 * =============================================================================
 */

#include "can_module.h"
#include "SPL/config.h"
#include <string.h>

/* ======================= Private Variables ======================= */
static CAN_SensorData_t sensor_data = {
    .temperature = 2500,        // 25.00°C (scaled by 100)
    .humidity = 6000,           // 60.00% (scaled by 100)
    .system_status = 0x01,      // System OK
    .transmission_counter = 0,
    .error_counter = 0,
    .last_error = CAN_ERROR_NONE
};

static CAN_Statistics_t statistics = {0};
static volatile uint8_t remote_request_pending = 0;
static volatile CAN_MessageID_t pending_remote_id = 0;

/* ======================= Private Function Prototypes ======================= */
static uint8_t CAN_WaitForTransmission(uint8_t mailbox);
static void CAN_IncrementTxCounter(void);
static void CAN_IncrementRxCounter(void);
static void CAN_IncrementErrorCounter(CAN_ErrorType_t error_type);

/* ======================= Public Functions Implementation ======================= */

/**
 * @brief Initialize CAN module with extended functionality
 */
void CAN_Module_Init(void)
{
    // Use existing config system
    CAN_Config();
    
    // Reset statistics
    CAN_ResetStatistics();
    
    // Initialize sensor data with default values
    sensor_data.temperature = 2500;  // 25.00°C
    sensor_data.humidity = 6000;     // 60.00%
    sensor_data.system_status = 0x01; // System OK
    sensor_data.transmission_counter = 0;
    sensor_data.error_counter = 0;
    sensor_data.last_error = CAN_ERROR_NONE;
}

/**
 * @brief Send CAN Data Frame
 */
uint8_t CAN_SendDataFrame(CAN_MessageID_t id, CAN_DataType_t data_type, uint8_t* data, uint8_t dlc)
{
    CanTxMsg TxMessage;
    uint8_t mailbox;
    
    // Configure message header
    TxMessage.StdId = id;
    TxMessage.ExtId = 0x00;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;        // Data Frame
    TxMessage.DLC = dlc;
    
    // Fill data based on type
    switch(data_type) {
        case CAN_DATA_TEMPERATURE:
            TxMessage.Data[0] = 'T';
            TxMessage.Data[1] = 'E';
            TxMessage.Data[2] = 'M';
            TxMessage.Data[3] = 'P';
            TxMessage.Data[4] = (sensor_data.temperature >> 8) & 0xFF;
            TxMessage.Data[5] = sensor_data.temperature & 0xFF;
            TxMessage.Data[6] = (sensor_data.transmission_counter >> 8) & 0xFF;
            TxMessage.Data[7] = sensor_data.transmission_counter & 0xFF;
            break;
            
        case CAN_DATA_HUMIDITY:
            TxMessage.Data[0] = 'H';
            TxMessage.Data[1] = 'U';
            TxMessage.Data[2] = 'M';
            TxMessage.Data[3] = 'I';
            TxMessage.Data[4] = (sensor_data.humidity >> 8) & 0xFF;
            TxMessage.Data[5] = sensor_data.humidity & 0xFF;
            TxMessage.Data[6] = (sensor_data.transmission_counter >> 8) & 0xFF;
            TxMessage.Data[7] = sensor_data.transmission_counter & 0xFF;
            break;
            
        case CAN_DATA_STATUS:
            TxMessage.Data[0] = 'S';
            TxMessage.Data[1] = 'T';
            TxMessage.Data[2] = 'A';
            TxMessage.Data[3] = 'T';
            TxMessage.Data[4] = sensor_data.system_status;
            TxMessage.Data[5] = (sensor_data.error_counter >> 8) & 0xFF;
            TxMessage.Data[6] = sensor_data.error_counter & 0xFF;
            TxMessage.Data[7] = sensor_data.last_error;
            break;
            
        case CAN_DATA_ERROR:
            // Custom data provided by user
            if (data != NULL) {
                for (uint8_t i = 0; i < dlc && i < 8; i++) {
                    TxMessage.Data[i] = data[i];
                }
            }
            break;
            
        default:
            return 0; // Invalid data type
    }
    
    // Transmit message
    mailbox = CAN_Transmit(CAN1, &TxMessage);
    if (mailbox == CAN_TxStatus_NoMailBox) {
        return 0; // Failed to get mailbox
    }
    
    // Wait for transmission completion
    if (CAN_WaitForTransmission(mailbox)) {
        CAN_IncrementTxCounter();
        sensor_data.transmission_counter++;
        return 1; // Success
    }
    
    return 0; // Transmission failed
}

/**
 * @brief Send CAN Remote Frame
 */
uint8_t CAN_SendRemoteFrame(CAN_MessageID_t id, uint8_t dlc)
{
    CanTxMsg TxMessage;
    uint8_t mailbox;
    
    // Configure remote frame
    TxMessage.StdId = id;
    TxMessage.ExtId = 0x00;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Remote;      // Remote Frame
    TxMessage.DLC = dlc;                 // Expected data length
    
    // Transmit remote frame
    mailbox = CAN_Transmit(CAN1, &TxMessage);
    if (mailbox == CAN_TxStatus_NoMailBox) {
        return 0; // Failed to get mailbox
    }
    
    // Wait for transmission completion
    if (CAN_WaitForTransmission(mailbox)) {
        statistics.remote_tx_count++;
        sensor_data.transmission_counter++;
        return 1; // Success
    }
    
    return 0; // Transmission failed
}

/**
 * @brief Handle Remote Frame Request
 */
void CAN_HandleRemoteRequest(CAN_MessageID_t remote_id)
{
    uint8_t response_sent = 0;
    
    switch(remote_id) {
        case CAN_ID_TEMPERATURE:
        case CAN_ID_REMOTE_TEMP:
            response_sent = CAN_SendDataFrame(CAN_ID_TEMPERATURE, CAN_DATA_TEMPERATURE, NULL, 8);
            break;
            
        case CAN_ID_HUMIDITY:
        case CAN_ID_REMOTE_HUM:
            response_sent = CAN_SendDataFrame(CAN_ID_HUMIDITY, CAN_DATA_HUMIDITY, NULL, 8);
            break;
            
        case CAN_ID_STATUS:
        case CAN_ID_REMOTE_STATUS:
            response_sent = CAN_SendDataFrame(CAN_ID_STATUS, CAN_DATA_STATUS, NULL, 8);
            break;
            
        default:
            // Unknown request - send error frame
            CAN_SendErrorFrame(CAN_ERROR_FORM);
            break;
    }
    
    if (response_sent) {
        statistics.remote_rx_count++;
    }
}

/**
 * @brief Send Error Frame (simulated)
 */
void CAN_SendErrorFrame(CAN_ErrorType_t error_type)
{
    uint8_t error_data[8];
    
    // Prepare error data
    error_data[0] = 'E';
    error_data[1] = 'R';
    error_data[2] = 'R';
    error_data[3] = 'O';
    error_data[4] = error_type;
    error_data[5] = (sensor_data.error_counter >> 8) & 0xFF;
    error_data[6] = sensor_data.error_counter & 0xFF;
    error_data[7] = 0xFF; // Error marker
    
    // Send as data frame with error ID
    CAN_SendDataFrame(CAN_ID_ERROR_NODE, CAN_DATA_ERROR, error_data, 8);
    
    // Update error tracking
    sensor_data.last_error = error_type;
    CAN_IncrementErrorCounter(error_type);
}

/**
 * @brief Process received CAN message
 */
void CAN_ProcessReceivedMessage(CanRxMsg* rx_msg)
{
    CAN_Message_t processed_msg;
    
    // Convert to internal message format
    processed_msg.id = rx_msg->StdId;
    processed_msg.dlc = rx_msg->DLC;
    processed_msg.timestamp = sensor_data.transmission_counter; // Use as timestamp
    
    for (uint8_t i = 0; i < rx_msg->DLC && i < 8; i++) {
        processed_msg.data[i] = rx_msg->Data[i];
    }
    
    // Determine frame type and process
    if (rx_msg->RTR == CAN_RTR_Remote) {
        processed_msg.frame_type = CAN_FRAME_REMOTE;
        statistics.remote_rx_count++;
        
        // Handle remote request
        CAN_HandleRemoteRequest((CAN_MessageID_t)rx_msg->StdId);
        
        // Call callback if defined
        CAN_RemoteFrameReceived_Callback((CAN_MessageID_t)rx_msg->StdId, rx_msg->DLC);
    } else {
        processed_msg.frame_type = CAN_FRAME_DATA;
        CAN_IncrementRxCounter();
        
        // Check if this is an error frame
        if (rx_msg->StdId == CAN_ID_ERROR_BUS || rx_msg->StdId == CAN_ID_ERROR_NODE) {
            processed_msg.frame_type = CAN_FRAME_ERROR;
            if (rx_msg->DLC >= 5) {
                CAN_ErrorType_t error_type = (CAN_ErrorType_t)rx_msg->Data[4];
                CAN_ErrorDetected_Callback(error_type);
            }
        } else {
            // Call data frame callback
            CAN_DataFrameReceived_Callback(&processed_msg);
        }
    }
}

/**
 * @brief Get current sensor data
 */
CAN_SensorData_t* CAN_GetSensorData(void)
{
    return &sensor_data;
}

/**
 * @brief Get CAN statistics
 */
CAN_Statistics_t* CAN_GetStatistics(void)
{
    return &statistics;
}

/**
 * @brief Update sensor data (simulate sensor readings)
 */
void CAN_UpdateSensorData(void)
{
    // Simulate temperature change (±2°C)
    static int8_t temp_direction = 1;
    sensor_data.temperature += temp_direction * 50; // 0.5°C steps
    if (sensor_data.temperature > 3500 || sensor_data.temperature < 1500) {
        temp_direction = -temp_direction;
    }
    
    // Simulate humidity change (±10%)
    static int8_t hum_direction = 1;
    sensor_data.humidity += hum_direction * 100; // 1% steps
    if (sensor_data.humidity > 9000 || sensor_data.humidity < 3000) {
        hum_direction = -hum_direction;
    }
}

/**
 * @brief Check for CAN errors and handle them
 */
void CAN_CheckAndHandleErrors(void)
{
    uint8_t esr = CAN1->ESR;
    
    if (esr & CAN_ESR_BOFF) {
        // Bus-off state
        CAN_SendErrorFrame(CAN_ERROR_BUS_OFF);
        statistics.bus_off_count++;
    } else if (esr & CAN_ESR_EPVF) {
        // Error passive state
        CAN_SendErrorFrame(CAN_ERROR_PASSIVE);
    }
    
    // Check for other error types based on ESR register
    uint8_t lec = (esr & CAN_ESR_LEC) >> 4;
    switch(lec) {
        case 1: CAN_SendErrorFrame(CAN_ERROR_STUFF); break;
        case 2: CAN_SendErrorFrame(CAN_ERROR_FORM); break;
        case 3: CAN_SendErrorFrame(CAN_ERROR_ACK); break;
        case 4: CAN_SendErrorFrame(CAN_ERROR_BIT_RECESSIVE); break;
        case 5: CAN_SendErrorFrame(CAN_ERROR_BIT_DOMINANT); break;
        case 6: CAN_SendErrorFrame(CAN_ERROR_CRC); break;
        default: break; // No error or undefined
    }
}

/**
 * @brief Reset CAN statistics
 */
void CAN_ResetStatistics(void)
{
    memset(&statistics, 0, sizeof(CAN_Statistics_t));
}

/**
 * @brief Format message for debugging output
 */
uint16_t CAN_FormatMessageForDebug(CAN_Message_t* msg, uint8_t* buffer, uint16_t buffer_size)
{
    uint16_t length = 0;
    const char* frame_types[] = {"DATA", "REMOTE", "ERROR"};
    
    // Add frame type
    const char* type_str = frame_types[msg->frame_type];
    for (uint8_t i = 0; type_str[i] && length < buffer_size - 1; i++) {
        buffer[length++] = type_str[i];
    }
    
    // Add ID
    const char* id_str = ": ID=0x";
    for (uint8_t i = 0; id_str[i] && length < buffer_size - 1; i++) {
        buffer[length++] = id_str[i];
    }
    
    // Convert ID to hex
    if (length + 3 < buffer_size) {
        buffer[length++] = ((msg->id >> 8) & 0x0F) < 10 ? 
                           '0' + ((msg->id >> 8) & 0x0F) : 
                           'A' + ((msg->id >> 8) & 0x0F) - 10;
        buffer[length++] = ((msg->id >> 4) & 0x0F) < 10 ? 
                           '0' + ((msg->id >> 4) & 0x0F) : 
                           'A' + ((msg->id >> 4) & 0x0F) - 10;
        buffer[length++] = (msg->id & 0x0F) < 10 ? 
                           '0' + (msg->id & 0x0F) : 
                           'A' + (msg->id & 0x0F) - 10;
    }
    
    if (msg->frame_type != CAN_FRAME_REMOTE) {
        // Add data for data frames
        const char* data_str = " DATA=[";
        for (uint8_t i = 0; data_str[i] && length < buffer_size - 1; i++) {
            buffer[length++] = data_str[i];
        }
        
        for (uint8_t i = 0; i < msg->dlc && length + 2 < buffer_size; i++) {
            buffer[length++] = (msg->data[i] >> 4) < 10 ? 
                               '0' + (msg->data[i] >> 4) : 
                               'A' + (msg->data[i] >> 4) - 10;
            buffer[length++] = (msg->data[i] & 0x0F) < 10 ? 
                               '0' + (msg->data[i] & 0x0F) : 
                               'A' + (msg->data[i] & 0x0F) - 10;
            if (i < msg->dlc - 1 && length < buffer_size - 1) {
                buffer[length++] = ' ';
            }
        }
        
        if (length < buffer_size - 1) {
            buffer[length++] = ']';
        }
    } else {
        // Add DLC for remote frames
        const char* dlc_str = " DLC=";
        for (uint8_t i = 0; dlc_str[i] && length < buffer_size - 1; i++) {
            buffer[length++] = dlc_str[i];
        }
        if (length < buffer_size - 1) {
            buffer[length++] = '0' + msg->dlc;
        }
    }
    
    // Add line ending
    if (length + 2 < buffer_size) {
        buffer[length++] = '\r';
        buffer[length++] = '\n';
    }
    
    buffer[length] = '\0';
    return length;
}

/* ======================= Private Functions Implementation ======================= */

static uint8_t CAN_WaitForTransmission(uint8_t mailbox)
{
    uint32_t timeout = 0xFFFF;
    while ((CAN_TransmitStatus(CAN1, mailbox) != CAN_TxStatus_Ok) && timeout--) {
        // Wait with timeout
    }
    return (timeout > 0) ? 1 : 0;
}

static void CAN_IncrementTxCounter(void)
{
    statistics.tx_count++;
}

static void CAN_IncrementRxCounter(void)
{
    statistics.rx_count++;
}

static void CAN_IncrementErrorCounter(CAN_ErrorType_t error_type)
{
    statistics.error_count++;
    sensor_data.error_counter++;
}

/* ======================= Weak Callback Functions ======================= */
__weak void CAN_DataFrameReceived_Callback(CAN_Message_t* msg)
{
    // Default implementation - can be overridden by user
    UNUSED(msg);
}

__weak void CAN_RemoteFrameReceived_Callback(CAN_MessageID_t id, uint8_t dlc)
{
    // Default implementation - can be overridden by user
    UNUSED(id);
    UNUSED(dlc);
}

__weak void CAN_ErrorDetected_Callback(CAN_ErrorType_t error_type)
{
    // Default implementation - can be overridden by user
    UNUSED(error_type);
}
