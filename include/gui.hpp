#pragma once

#include "display/lvgl.h"

#include "libraidzero/gui/odomDebug.hpp"

#include <memory>

class GUI {
public:
    std::string selectedAuton;

    static GUI& getInstance();

    GUI(GUI const&) = delete;
    void operator=(GUI const&) = delete;

    void initOdom();
    void initMenu();

    void setState(OdomDebug::state_t state);
    void resetSensors();

    void setData(const OdomDebug::state_t& state, 
        const OdomDebug::sensors_t& sensors);

private:
    GUI();

    std::unique_ptr<OdomDebug> odomDisplay{nullptr};

    lv_style_t backgroundStyle;
    lv_style_t buttonStyleREL, buttonStylePR;
    lv_style_t buttonMatrixStyle;
    lv_style_t rightAutonStyleREL, rightAutonStylePR;
    lv_obj_t* openOdomButton;
    lv_obj_t* openOdomButtonLabel;
    lv_obj_t* rightAutonButtonMatrix;
    lv_obj_t* selectedAutonLabel;

    static lv_res_t openOdomAction(lv_obj_t* button);
    static lv_res_t selectRightAutonAction(lv_obj_t* buttonMatrix, const char* text);
};
