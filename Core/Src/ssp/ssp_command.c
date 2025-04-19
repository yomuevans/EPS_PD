#include "ssp_command.h"
#include "ssp_frame.h"
#include "ssp_comm.h"
#include "ssp_power.h"
#include "eps_faults.h"
#include <stdbool.h>
#include <string.h>

// State Variables
static uint8_t CurrentMode = SSP_PARAM_INITIALIZE;
static uint32_t SynchCounter = 0;
static SSP_FunctionParam FunctionParams[] = {
    {SSP_FUNC_ID_BATTERY_MONITOR, 0},
    {SSP_FUNC_ID_POWER_REGULATION, 0}
};
static uint8_t FunctionParamCount = sizeof(FunctionParams) / sizeof(FunctionParams[0]);
static bool MissionTerminationEnabled = false;

void SSP_HandleCommand(SSP_Frame *rx_frame, SSP_Frame *tx_frame, UART_HandleTypeDef *huart,
                       UART_HandleTypeDef *huart_log, GPIO_TypeDef *de_port, uint16_t de_pin)
{
    uint8_t src_addr = rx_frame->SrcAddr;
    uint8_t dest_addr = rx_frame->DestAddr;
    uint8_t cmd_id = rx_frame->CmdId;
    uint8_t data_len = rx_frame->DataLen;
    uint8_t *data = rx_frame->Data;

    uint8_t response_cmd = SSP_RESP_ACK;
    uint8_t response_data[SSP_MAX_DATA_LEN];
    uint8_t response_len = 1;

    // Check for faults (already handled via interrupts/polling)
    // Proceed with command processing if no faults interfere
    switch (cmd_id) {
        case SSP_CMD_PING:
            response_data[0] = SSP_CMD_PING;
            break;

        case SSP_CMD_GD:
            if (data_len >= 1) {
                uint8_t param_id = data[0];
                uint8_t param_count;
                SSP_Parameter *params = SSP_GetParameters(&param_count);
                for (uint8_t i = 0; i < param_count; i++) {
                    if (params[i].ParamId == param_id) {
                        response_data[0] = cmd_id;
                        response_data[1] = param_id;
                        response_data[2] = (params[i].Value >> 8) & 0xFF;
                        response_data[3] = params[i].Value & 0xFF;
                        response_len = 4;
                        break;
                    }
                }
                if (response_len == 1) {
                    response_cmd = SSP_RESP_NACK;
                    response_data[0] = cmd_id;
                }
            } else {
                response_cmd = SSP_RESP_NACK;
                response_data[0] = cmd_id;
            }
            break;

        case SSP_CMD_PD:
            if (data_len >= 3) {
                uint8_t param_id = data[0];
                uint16_t value = (data[1] << 8) | data[2];
                uint8_t param_count;
                SSP_Parameter *params = SSP_GetParameters(&param_count);
                for (uint8_t i = 0; i < param_count; i++) {
                    if (params[i].ParamId == param_id) {
                        params[i].Value = value;
                        response_data[0] = cmd_id;
                        break;
                    }
                }
                if (response_len == 1) {
                    response_cmd = SSP_RESP_NACK;
                    response_data[0] = cmd_id;
                }
            } else {
                response_cmd = SSP_RESP_NACK;
                response_data[0] = cmd_id;
            }
            break;

        case SSP_CMD_RD:
            if (data_len >= 1) {
                uint8_t param_id = data[0];
                uint8_t param_count;
                SSP_Parameter *params = SSP_GetParameters(&param_count);
                for (uint8_t i = 0; i < param_count; i++) {
                    if (params[i].ParamId == param_id) {
                        response_data[0] = cmd_id;
                        response_data[1] = param_id;
                        response_data[2] = (params[i].Value >> 8) & 0xFF;
                        response_data[3] = params[i].Value & 0xFF;
                        response_len = 4;
                        break;
                    }
                }
                if (response_len == 1) {
                    response_cmd = SSP_RESP_NACK;
                    response_data[0] = cmd_id;
                }
            } else {
                response_cmd = SSP_RESP_NACK;
                response_data[0] = cmd_id;
            }
            break;

        case SSP_CMD_WD:
            if (data_len >= 3) {
                uint8_t param_id = data[0];
                uint16_t value = (data[1] << 8) | data[2];
                uint8_t param_count;
                SSP_Parameter *params = SSP_GetParameters(&param_count);
                for (uint8_t i = 0; i < param_count; i++) {
                    if (params[i].ParamId == param_id) {
                        params[i].Value = value;
                        response_data[0] = cmd_id;
                        break;
                    }
                }
                if (response_len == 1) {
                    response_cmd = SSP_RESP_NACK;
                    response_data[0] = cmd_id;
                }
            } else {
                response_cmd = SSP_RESP_NACK;
                response_data[0] = cmd_id;
            }
            break;

        case SSP_CMD_SON:
        case SSP_CMD_SOF:
            if (dest_addr == SSP_ADDR_EPS || dest_addr == SSP_ADDR_ALL) {
                if (data_len >= 1) {
                    uint8_t power_line = data[0];
                    if (cmd_id == SSP_CMD_SON) {
                        SSP_SetPowerLine(power_line);
                    } else {
                        SSP_DisablePowerLine(power_line);
                    }
                    response_data[0] = cmd_id;
                } else {
                    response_cmd = SSP_RESP_NACK;
                    response_data[0] = cmd_id;
                }
            } else {
                response_cmd = SSP_RESP_NACK;
                response_data[0] = cmd_id;
            }
            break;

        case SSP_CMD_SM:
            if (dest_addr == SSP_ADDR_ALL) {
                if (data_len >= 1) {
                    uint8_t mode = data[0];
                    switch (mode) {
                        case SSP_PARAM_INITIALIZE:
                            SSP_SetPowerLine(SSP_PARAM_CCU5V_EN);
                            break;
                        case SSP_PARAM_DETUMBLE:
                            SSP_SetPowerLine(SSP_PARAM_CCU5V_EN);
                            SSP_SetPowerLine(SSP_PARAM_GPS_EN);
                            SSP_SetPowerLine(SSP_PARAM_ADCS12V_EN);
                            break;
                        case SSP_PARAM_NORMAL:
                            SSP_SetPowerLine(SSP_PARAM_GPS_EN);
                            SSP_SetPowerLine(SSP_PARAM_ADCS5V_EN);
                            SSP_SetPowerLine(SSP_PARAM_RS3V3_EN);
                            SSP_SetPowerLine(SSP_PARAM_UHF_EN);
                            break;
                        case SSP_PARAM_COMMUNICATION:
                            SSP_SetPowerLine(SSP_PARAM_GPS_EN);
                            SSP_SetPowerLine(SSP_PARAM_RS3V3_EN);
                            SSP_SetPowerLine(SSP_PARAM_ADCS5V_EN);
                            SSP_SetPowerLine(SSP_PARAM_ADCS12V_EN);
                            SSP_SetPowerLine(SSP_PARAM_UHF_EN);
                            break;
                        case SSP_PARAM_PAYLOAD:
                            SSP_SetPowerLine(SSP_PARAM_RS3V3_EN);
                            SSP_SetPowerLine(SSP_PARAM_ADCS5V_EN);
                            SSP_SetPowerLine(SSP_PARAM_ADCS12V_EN);
                            SSP_SetPowerLine(SSP_PARAM_GPS_EN);
                            SSP_SetPowerLine(SSP_PARAM_PL5V_EN);
                            SSP_SetPowerLine(SSP_PARAM_UHF_EN);
                            break;
                        case SSP_PARAM_IMAGE:
                            SSP_SetPowerLine(SSP_PARAM_GPS_EN);
                            SSP_SetPowerLine(SSP_PARAM_ADCS5V_EN);
                            SSP_SetPowerLine(SSP_PARAM_ADCS12V_EN);
                            SSP_SetPowerLine(SSP_PARAM_PL5V_EN);
                            SSP_SetPowerLine(SSP_PARAM_XB12V_EN);
                            SSP_SetPowerLine(SSP_PARAM_UHF_EN);
                            break;
                        case SSP_PARAM_EMERGENCY:
                            SSP_SetPowerLine(SSP_PARAM_ADCS12V_EN);
                            SSP_SetPowerLine(SSP_PARAM_ADCS5V_EN);
                            SSP_SetPowerLine(SSP_PARAM_GPS_EN);
                            SSP_SetPowerLine(SSP_PARAM_UHF_EN);
                            break;
                        default:
                            response_cmd = SSP_RESP_NACK;
                            break;
                    }
                    if (response_cmd == SSP_RESP_ACK) {
                        CurrentMode = mode;
                        response_data[0] = cmd_id;
                    }
                } else {
                    response_cmd = SSP_RESP_NACK;
                    response_data[0] = cmd_id;
                }
            } else {
                response_cmd = SSP_RESP_NACK;
                response_data[0] = cmd_id;
            }
            break;

        case SSP_CMD_GM:
            response_data[0] = cmd_id;
            response_data[1] = CurrentMode;
            response_len = 2;
            break;

        case SSP_CMD_GSC:
            response_data[0] = cmd_id;
            response_data[1] = (SynchCounter >> 24) & 0xFF;
            response_data[2] = (SynchCounter >> 16) & 0xFF;
            response_data[3] = (SynchCounter >> 8) & 0xFF;
            response_data[4] = SynchCounter & 0xFF;
            response_len = 5;
            break;

        case SSP_CMD_SSC:
            if (data_len >= 4) {
                SynchCounter = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
                response_data[0] = cmd_id;
            } else {
                response_cmd = SSP_RESP_NACK;
                response_data[0] = cmd_id;
            }
            break;

        case SSP_CMD_GFP:
            if (data_len >= 1) {
                uint8_t func_id = data[0];
                for (uint8_t i = 0; i < FunctionParamCount; i++) {
                    if (FunctionParams[i].FuncId == func_id) {
                        response_data[0] = cmd_id;
                        response_data[1] = func_id;
                        response_data[2] = (FunctionParams[i].Value >> 8) & 0xFF;
                        response_data[3] = FunctionParams[i].Value & 0xFF;
                        response_len = 4;
                        break;
                    }
                }
                if (response_len == 1) {
                    response_cmd = SSP_RESP_NACK;
                    response_data[0] = cmd_id;
                }
            } else {
                response_cmd = SSP_RESP_NACK;
                response_data[0] = cmd_id;
            }
            break;

        case SSP_CMD_SFP:
            if (data_len >= 3) {
                uint8_t func_id = data[0];
                uint16_t value = (data[1] << 8) | data[2];
                for (uint8_t i = 0; i < FunctionParamCount; i++) {
                    if (FunctionParams[i].FuncId == func_id) {
                        FunctionParams[i].Value = value;
                        response_data[0] = cmd_id;
                        break;
                    }
                }
                if (response_len == 1) {
                    response_cmd = SSP_RESP_NACK;
                    response_data[0] = cmd_id;
                }
            } else {
                response_cmd = SSP_RESP_NACK;
                response_data[0] = cmd_id;
            }
            break;

        case SSP_CMD_FON:
            if (data_len >= 1) {
                uint8_t func_id = data[0];
                for (uint8_t i = 0; i < FunctionParamCount; i++) {
                    if (FunctionParams[i].FuncId == func_id) {
                        FunctionParams[i].Value = 1;
                        response_data[0] = cmd_id;
                        break;
                    }
                }
                if (response_len == 1) {
                    response_cmd = SSP_RESP_NACK;
                    response_data[0] = cmd_id;
                }
            } else {
                response_cmd = SSP_RESP_NACK;
                response_data[0] = cmd_id;
            }
            break;

        case SSP_CMD_FOF:
            if (data_len >= 1) {
                uint8_t func_id = data[0];
                for (uint8_t i = 0; i < FunctionParamCount; i++) {
                    if (FunctionParams[i].FuncId == func_id) {
                        FunctionParams[i].Value = 0;
                        response_data[0] = cmd_id;
                        break;
                    }
                }
                if (response_len == 1) {
                    response_cmd = SSP_RESP_NACK;
                    response_data[0] = cmd_id;
                }
            } else {
                response_cmd = SSP_RESP_NACK;
                response_data[0] = cmd_id;
            }
            break;

        case SSP_CMD_GOSTM:
            {
                SSP_Telemetry *telemetry = SSP_GetTelemetry();
                response_data[0] = cmd_id;
                response_data[1] = (telemetry->Bus12V >> 8) & 0xFF;
                response_data[2] = telemetry->Bus12V & 0xFF;
                response_data[3] = (telemetry->Bus5V >> 8) & 0xFF;
                response_data[4] = telemetry->Bus5V & 0xFF;
                response_data[5] = (telemetry->Bus3V3 >> 8) & 0xFF;
                response_data[6] = telemetry->Bus3V3 & 0xFF;
                response_len = 7;
            }
            break;

        case SSP_CMD_KEN:
            if (dest_addr == SSP_ADDR_EPS) {
                MissionTerminationEnabled = true;
                response_data[0] = cmd_id;
                char log_msg[] = "Mission_Termination_Enabled\n";
                HAL_UART_Transmit(huart_log, (uint8_t*)log_msg, strlen(log_msg), 100);
            } else {
                response_cmd = SSP_RESP_NACK;
                response_data[0] = cmd_id;
            }
            break;

        case SSP_CMD_KDIS:
            if (dest_addr == SSP_ADDR_EPS) {
                MissionTerminationEnabled = false;
                response_data[0] = cmd_id;
                char log_msg[] = "Mission_Termination_Disabled\n";
                HAL_UART_Transmit(huart_log, (uint8_t*)log_msg, strlen(log_msg), 100);
            } else {
                response_cmd = SSP_RESP_NACK;
                response_data[0] = cmd_id;
            }
            break;

        default:
            response_cmd = SSP_RESP_NACK;
            response_data[0] = cmd_id;
            char log_msg[] = "Unknown_Command_Received\n";
            HAL_UART_Transmit(huart_log, (uint8_t*)log_msg, strlen(log_msg), 100);
            break;
    }

    SSP_PackFrame(tx_frame, SSP_ADDR_EPS, src_addr, response_cmd, response_data, response_len);
    SSP_SendFrame(huart, de_port, de_pin, tx_frame);
}
