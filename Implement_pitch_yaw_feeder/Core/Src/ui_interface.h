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

/* Dropdown pages: Launcher, Pitch, Yaw, RC, Auto (match Subsystem in vars.h) */
typedef enum {
    SELECT_NONE = 0,
    SELECT_PITCH_ANGLE,
    SELECT_YAW_ANGLE,
    SELECT_LAUNCHER_FORCE,
    SELECT_RC,   /* RC page - no value edit */
    SELECT_AUTO  /* Auto page - no value edit */
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
    volatile bool fire_request;
    volatile bool draw_request;

    /* Firing state - native variable for UI (get_var_firing / set_var_firing) */
    volatile bool firing;

    /* Auto page: number of darts to shoot (set by matrix button, 1..9) */
    uint8_t auto_dart_count;
} ui_control_state_t;

/* Global UI control state - shared between UI and tasks */
extern ui_control_state_t ui_state;

/* ========================= UI Interface Functions ========================= */

/**
 * @brief Initialize the UI interface state
 */
void ui_interface_init(void);

/**
 * @brief Check if UI requested a redraw and clear the flag
 * @return true if draw was requested
 */
bool ui_interface_check_draw_request(void);

/**
 * @brief Request UI to redraw
 */
void ui_interface_trigger_draw(void);

/**
 * @brief Trigger a fire request (from UI Fire button)
 */
void ui_interface_trigger_fire(void);

/**
 * @brief Check if fire was requested and clear the flag
 * @return true if fire was requested
 */
bool ui_interface_check_fire_request(void);

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

/**
 * @brief Set the firing state (for native variable; also call from launcher task)
 * @param firing true when system is firing
 */
void ui_interface_set_firing(bool firing);

/**
 * @brief Get the firing state
 * @return true when system is firing
 */
bool ui_interface_get_firing(void);

/* ========================= Auto page: dart count ========================= */

/**
 * @brief Set number of darts to shoot in Auto mode (from matrix button, 1..9)
 * @param count Number of darts (1 to 9)
 */
void ui_interface_set_auto_dart_count(uint8_t count);

/**
 * @brief Get number of darts to shoot in Auto mode
 * @return Number of darts (1 to 9)
 */
uint8_t ui_interface_get_auto_dart_count(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_UI_INTERFACE_H_ */
