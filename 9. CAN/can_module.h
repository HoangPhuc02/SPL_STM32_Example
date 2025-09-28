/*
 * =============================================================================
 * Project: STM32F103 CAN Module Header
 * File: can_module.h
 * Author: hoangphuc540202@gmail.com
 * Date: August 2025
 * 
 * Description: Comprehensive CAN module with Data Frame, Remote Frame, 
 *              and Error Frame handling
 * =============================================================================
 */

#ifndef CAN_MODULE_H
#define CAN_MODULE_H

#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_can.h"

/* ======================= CAN Message Type Definitions ======================= */
typedef enum {
    CAN_FRAME_DATA = 0,
    CAN_FRAME_REMOTE,
    CAN_FRAME_ERROR
} CAN_FrameType_t;

typedef enum {
    CAN_ID_TEMPERATURE   = 0x101,  // Temperature sensor data
    CAN_ID_HUMIDITY      = 0x102,  // Humidity sensor data
    CAN_ID_STATUS        = 0x103,  // System status
    CAN_ID_REMOTE_TEMP   = 0x201,  // Remote request for temperature
    CAN_ID_REMOTE_HUM    = 0x202,  // Remote request for humidity
    CAN_ID_REMOTE_STATUS = 0x203,  // Remote request for status
    CAN_ID_ERROR_BUS     = 0x301,  // Bus error
    CAN_ID_ERROR_NODE    = 0x302   // Node error
} CAN_MessageID_t;

typedef enum {
    CAN_DATA_TEMPERATURE = 0,
    CAN_DATA_HUMIDITY,
    CAN_DATA_STATUS,
    CAN_DATA_ERROR
} CAN_DataType_t;

typedef enum {
    CAN_ERROR_NONE = 0,
    CAN_ERROR_STUFF,        // Stuff error
    CAN_ERROR_FORM,         // Form error
    CAN_ERROR_ACK,          // Acknowledgment error
    CAN_ERROR_BIT_RECESSIVE,// Bit error (recessive)
    CAN_ERROR_BIT_DOMINANT, // Bit error (dominant)
    CAN_ERROR_CRC,          // CRC error
    CAN_ERROR_BUS_OFF,      // Bus-off state
    CAN_ERROR_PASSIVE       // Error passive state
} CAN_ErrorType_t;

/* ======================= Data Structures ======================= */
typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
    CAN_FrameType_t frame_type;
    uint32_t timestamp;
} CAN_Message_t;

typedef struct {
    uint32_t temperature;
    uint32_t humidity;
    uint8_t system_status;
    uint32_t transmission_counter;
    uint32_t error_counter;
    CAN_ErrorType_t last_error;
} CAN_SensorData_t;

typedef struct {
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t remote_tx_count;
    uint32_t remote_rx_count;
    uint32_t error_count;
    uint32_t bus_off_count;
} CAN_Statistics_t;

/* ======================= Function Prototypes ======================= */

/**
 * @brief Initialize CAN module with extended functionality
 */
void CAN_Module_Init(void);

/**
 * @brief Send CAN Data Frame
 * @param id: Message ID
 * @param data_type: Type of data to send
 * @param data: Pointer to data array
 * @param dlc: Data length code
 * @return Success status
 */
uint8_t CAN_SendDataFrame(CAN_MessageID_t id, CAN_DataType_t data_type, uint8_t* data, uint8_t dlc);

/**
 * @brief Send CAN Remote Frame
 * @param id: Message ID to request
 * @param dlc: Expected data length
 * @return Success status
 */
uint8_t CAN_SendRemoteFrame(CAN_MessageID_t id, uint8_t dlc);

/**
 * @brief Handle Remote Frame Request
 * @param remote_id: ID from received remote frame
 */
void CAN_HandleRemoteRequest(CAN_MessageID_t remote_id);

/**
 * @brief Send Error Frame (simulated)
 * @param error_type: Type of error
 */
void CAN_SendErrorFrame(CAN_ErrorType_t error_type);

/**
 * @brief Process received CAN message
 * @param rx_msg: Pointer to received message structure
 */
void CAN_ProcessReceivedMessage(CanRxMsg* rx_msg);

/**
 * @brief Get current sensor data
 * @return Pointer to sensor data structure
 */
CAN_SensorData_t* CAN_GetSensorData(void);

/**
 * @brief Get CAN statistics
 * @return Pointer to statistics structure
 */
CAN_Statistics_t* CAN_GetStatistics(void);

/**
 * @brief Update sensor data (simulate sensor readings)
 */
void CAN_UpdateSensorData(void);

/**
 * @brief Check for CAN errors and handle them
 */
void CAN_CheckAndHandleErrors(void);

/**
 * @brief Reset CAN statistics
 */
void CAN_ResetStatistics(void);

/**
 * @brief Format message for debugging output
 * @param msg: Pointer to CAN message
 * @param buffer: Output buffer
 * @param buffer_size: Size of output buffer
 * @return Length of formatted string
 */
uint16_t CAN_FormatMessageForDebug(CAN_Message_t* msg, uint8_t* buffer, uint16_t buffer_size);

/* ======================= Callback Function Prototypes ======================= */
extern void CAN_DataFrameReceived_Callback(CAN_Message_t* msg);
extern void CAN_RemoteFrameReceived_Callback(CAN_MessageID_t id, uint8_t dlc);
extern void CAN_ErrorDetected_Callback(CAN_ErrorType_t error_type);

#endif /* CAN_MODULE_H */
