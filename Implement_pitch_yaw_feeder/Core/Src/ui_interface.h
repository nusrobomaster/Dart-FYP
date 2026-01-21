/*
 * ui_interface.h
 *
 * Interface between UI and subsystem tasks (pitch, yaw, launcher, feeder)
 * This file should be included by both UI code and task code
 *
 * Created on: Jan 21, 2026
 */

#ifndef SRC_UI_INTERFACE_H_
#define SRC_UI_INTERFACE_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= Selection Modes ========================= */

typedef enum {
    SELECT_NONE = 0,
    SELECT_PITCH_ANGLE,
    SELECT_YAW_ANGLE,
    SELECT_LAUNCHER_FORCE
} ui_selection_mode_t;

/* ========================= UI Control State ========================= */

typedef struct {
    /* Current selection mode - which parameter is being edited */
    ui_selection_mode_t selection_mode;
    
    /* Set values (targets) - these are applied when OK is pressed */
    float set_pitch_angle;
    float set_yaw_angle;
    float set_launcher_force;
    
    /* Current/actual values from sensors */
    float current_pitch_angle;
    float current_yaw_angle;
    float current_launcher_force;
    
    /* Flags for subsystem communication */
    volatile bool feeder_reload_request;  /* Set true to trigger reload */
    volatile bool values_updated;          /* Set true when OK is pressed */
    
} ui_control_state_t;

/* Global UI control state - shared between UI and tasks */
extern ui_control_state_t ui_state;

/* ========================= UI Interface Functions ========================= */

/**
 * @brief Initialize the UI interface state
 */
void ui_interface_init(void);

/**
 * @brief Handle number pad button press
 * @param btn_text Text of the pressed button ("0"-"9", "DEL", "OK")
 */
void ui_interface_numpad_press(const char *btn_text);

/**
 * @brief Set the selection mode (which parameter to edit)
 * @param mode The selection mode
 */
void ui_interface_set_selection(ui_selection_mode_t mode);

/**
 * @brief Get current selection mode
 * @return Current selection mode
 */
ui_selection_mode_t ui_interface_get_selection(void);

/**
 * @brief Trigger feeder reload
 */
void ui_interface_trigger_reload(void);

/**
 * @brief Check if feeder reload was requested and clear the flag
 * @return true if reload was requested
 */
bool ui_interface_check_reload_request(void);

/**
 * @brief Apply the current input value to the selected parameter
 */
void ui_interface_apply_value(void);

/**
 * @brief Apply a value directly to a specific parameter
 * @param mode The selection mode (which parameter to set)
 * @param value The value to apply
 */
void ui_interface_apply_value_direct(ui_selection_mode_t mode, float value);

/**
 * @brief Get the current input string from textarea
 * @return Pointer to the input string
 */
const char* ui_interface_get_input_text(void);

/**
 * @brief Clear the input textarea
 */
void ui_interface_clear_input(void);

/**
 * @brief Update current sensor values (call from tasks)
 * @param pitch_angle Current pitch angle in degrees
 * @param yaw_angle Current yaw angle in degrees
 */
void ui_interface_update_current_values(float pitch_angle, float yaw_angle);

/**
 * @brief Get the set pitch angle value
 * @return Set pitch angle in degrees
 */
float ui_interface_get_set_pitch(void);

/**
 * @brief Get the set yaw angle value
 * @return Set yaw angle in degrees
 */
float ui_interface_get_set_yaw(void);

/**
 * @brief Get the set launcher force value
 * @return Set launcher force
 */
float ui_interface_get_set_force(void);

/* ========================= UI Update Callbacks ========================= */

/**
 * @brief Update the UI display (call from UI tick)
 * Updates labels with current values and highlights selected button
 */
void ui_interface_update_display(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_UI_INTERFACE_H_ */
