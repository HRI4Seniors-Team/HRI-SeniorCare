#ifndef __GIMBAL_CONTROLLER_H__
#define __GIMBAL_CONTROLLER_H__

#include "mcp_server.h"
#include "uart_k210.h"
#include <esp_log.h>

class GimbalController {
public:
    GimbalController(UartK210& uart_k210) : uart_k210_(uart_k210) {
        TAG = "GimbalController";
        RegisterMcpTools();
    }

    void RegisterMcpTools() {
        auto& mcp_server = McpServer::GetInstance();
        
        // 获取云台状态
        mcp_server.AddTool("gimbal.get_state", 
            "Get the current state of the gimbal (pitch and roll servo positions)", 
            PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                SendCommand("GET_STATE");
                ESP_LOGI(TAG, "Request gimbal state");
                return true;
            });

        // 横滚轴左转
        mcp_server.AddTool("gimbal.roll.turn_left", 
            "Turn the roll servo to the left", 
            PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                SendCommand("ROLL_LEFT");
                ESP_LOGI(TAG, "Roll left");
                return true;
            });

        // 横滚轴右转
        mcp_server.AddTool("gimbal.roll.turn_right", 
            "Turn the roll servo to the right", 
            PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                SendCommand("ROLL_RIGHT");
                ESP_LOGI(TAG, "Roll right");
                return true;
            });

        // 俯仰轴上转
        mcp_server.AddTool("gimbal.pitch.turn_up", 
            "Turn the pitch servo up", 
            PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                SendCommand("PITCH_UP");
                ESP_LOGI(TAG, "Pitch up");
                return true;
            });

        // 俯仰轴下转
        mcp_server.AddTool("gimbal.pitch.turn_down", 
            "Turn the pitch servo down", 
            PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                SendCommand("PITCH_DOWN");
                ESP_LOGI(TAG, "Pitch down");
                return true;
            });

        // 保持当前位置
        mcp_server.AddTool("gimbal.hold_position", 
            "Keep the servo in its current position (stop audio tracking)", 
            PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                SendCommand("HOLD_POSITION");
                ESP_LOGI(TAG, "Hold position");
                return true;
            });

        // 回归初始位置
        mcp_server.AddTool("gimbal.reset", 
            "Reset the gimbal to initial position", 
            PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                SendCommand("RESET");
                ESP_LOGI(TAG, "Reset to initial position");
                return true;
            });

        // 启用音频跟踪
        mcp_server.AddTool("gimbal.enable_tracking", 
            "Enable audio source tracking", 
            PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                SendCommand("ENABLE_TRACKING");
                ESP_LOGI(TAG, "Enable audio tracking");
                return true;
            });

        // 禁用音频跟踪
        mcp_server.AddTool("gimbal.disable_tracking", 
            "Disable audio source tracking", 
            PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                SendCommand("DISABLE_TRACKING");
                ESP_LOGI(TAG, "Disable audio tracking");
                return true;
            });
    }

private:
    UartK210& uart_k210_;
    const char* TAG;

    // 发送命令到 K210
    void SendCommand(const char* cmd) {
        std::string message = std::string(cmd) + "\n";
        uart_k210_.SendData(message.c_str(), message.length());
        ESP_LOGI(TAG, "Sent to K210: %s", cmd);
    }
};

#endif // __GIMBAL_CONTROLLER_H__