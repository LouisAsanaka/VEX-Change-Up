#include "gui.hpp"

#include "okapi/api/units/QAngle.hpp"

GUI& GUI::getInstance() {
    static GUI instance;
    return instance;
}

GUI::GUI() {
    
}

void GUI::initOdom() {
    odomDisplay = std::make_unique<OdomDebug>(lv_scr_act(), LV_COLOR_ORANGE);
    odomDisplay->setStateCallback([&](OdomDebug::state_t state) {setState(state);});
    odomDisplay->setResetCallback([&]() {resetSensors();});
}

void GUI::initMenu() {
    lv_coord_t screenWidth = lv_obj_get_width(lv_scr_act());
    lv_coord_t screenHeight = lv_obj_get_height(lv_scr_act());

    // Background color
    lv_style_copy(&backgroundStyle, &lv_style_plain_color);
    backgroundStyle.body.main_color = LV_COLOR_BLACK;
    backgroundStyle.body.grad_color = LV_COLOR_BLACK;
    backgroundStyle.body.border.width = 0;
    backgroundStyle.body.radius = 0;
    lv_obj_set_style(lv_scr_act(), &backgroundStyle);

    // Background image
    //LV_IMG_DECLARE(raidzero);
    //lv_obj_t* logo = lv_img_create(lv_scr_act(), NULL);
    //lv_img_set_src(logo, &raidzero);
    //lv_obj_align(logo, NULL, LV_ALIGN_CENTER, 0, 0);

    // Button Styles
    lv_style_copy(&buttonStyleREL, &lv_style_plain);
    buttonStyleREL.body.main_color = LV_COLOR_MAKE(39, 174, 96);
    buttonStyleREL.body.grad_color = buttonStyleREL.body.main_color;
    buttonStyleREL.body.radius = 4;
    buttonStyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

    lv_style_copy(&buttonStylePR, &lv_style_plain);
    buttonStylePR.body.main_color = LV_COLOR_MAKE(46, 204, 113);
    buttonStylePR.body.grad_color = buttonStylePR.body.main_color;
    buttonStylePR.body.radius = 4;
    buttonStylePR.text.color = LV_COLOR_MAKE(255, 255, 255);

    lv_style_copy(&buttonMatrixStyle, &lv_style_plain_color);
    buttonMatrixStyle.body.main_color = LV_COLOR_BLACK;
    buttonMatrixStyle.body.grad_color = buttonMatrixStyle.body.main_color;
    buttonMatrixStyle.body.padding.ver = 3;

    lv_style_copy(&rightAutonStyleREL, &buttonStyleREL);
    rightAutonStyleREL.body.main_color = LV_COLOR_MAKE(211, 84, 0);
    rightAutonStyleREL.body.grad_color = rightAutonStyleREL.body.main_color;
    rightAutonStyleREL.body.radius = 4;
    rightAutonStyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

    lv_style_copy(&rightAutonStylePR, &buttonStylePR);
    rightAutonStylePR.body.main_color = LV_COLOR_MAKE(230, 126, 34);
    rightAutonStylePR.body.grad_color = rightAutonStylePR.body.main_color;
    rightAutonStylePR.body.radius = 4;
    rightAutonStylePR.text.color = LV_COLOR_MAKE(255, 255, 255);

    // Open OdomDebug button
    openOdomButton = lv_btn_create(lv_scr_act(), NULL);
    lv_obj_set_free_num(openOdomButton, 0);
    lv_obj_set_free_ptr(openOdomButton, this);
    lv_btn_set_action(openOdomButton, LV_BTN_ACTION_CLICK, openOdomAction);
    lv_btn_set_style(openOdomButton, LV_BTN_STYLE_REL, &buttonStyleREL);
    lv_btn_set_style(openOdomButton, LV_BTN_STYLE_PR, &buttonStylePR);
    lv_obj_set_size(openOdomButton, 200, 40);
    lv_obj_align(openOdomButton, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);

    openOdomButtonLabel = lv_label_create(openOdomButton, NULL);
    lv_label_set_text(openOdomButtonLabel, "Open OdomDebug");

    // Red Side Autons
    static const char* rightAutonButtonMap[] = { 
        "\222Right 1", "\n",
        "\222Right 2", "\n",
        "\222Right 3", ""
    };

    rightAutonButtonMatrix = lv_btnm_create(lv_scr_act(), NULL);
    lv_obj_set_free_ptr(rightAutonButtonMatrix, this);
    lv_obj_set_size(rightAutonButtonMatrix, 120, screenHeight - 20);
    lv_obj_align(rightAutonButtonMatrix, NULL, LV_ALIGN_IN_LEFT_MID, 10, 5);
    lv_btnm_set_map(rightAutonButtonMatrix, rightAutonButtonMap);
    lv_btnm_set_action(rightAutonButtonMatrix, selectRightAutonAction);
    lv_btnm_set_style(rightAutonButtonMatrix, LV_BTNM_STYLE_BG, &buttonMatrixStyle);
    lv_btnm_set_style(rightAutonButtonMatrix, LV_BTNM_STYLE_BTN_REL, &rightAutonStyleREL);
    lv_btnm_set_style(rightAutonButtonMatrix, LV_BTNM_STYLE_BTN_PR, &rightAutonStylePR);

    // Auton selection label
    selectedAutonLabel = lv_label_create(lv_scr_act(), NULL);
    lv_label_set_text(selectedAutonLabel, "No auton selected");
    lv_obj_align(selectedAutonLabel, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -10);
}

void GUI::setState(OdomDebug::state_t state) {

}

void GUI::resetSensors() {

}

void GUI::setData(const OdomDebug::state_t& state, 
    const OdomDebug::sensors_t& sensors) 
{
    if (odomDisplay != nullptr) {
        odomDisplay->setData(state, sensors);
    }
}

lv_res_t GUI::openOdomAction(lv_obj_t* button) {
    GUI* that = static_cast<GUI*>(lv_obj_get_free_ptr(button));
    that->initOdom();
    return LV_RES_OK;
}

lv_res_t GUI::selectRightAutonAction(lv_obj_t* buttonMatrix, const char* text) {
    GUI* that = static_cast<GUI*>(lv_obj_get_free_ptr(buttonMatrix));
    std::string cppStr{text};
    if (that->selectedAuton == cppStr) { // Click the auton again to remove
        that->selectedAuton = "No auton selected";
        lv_label_set_text(that->selectedAutonLabel, that->selectedAuton.c_str());
    } else {
        that->selectedAuton = cppStr;
        lv_label_set_text(that->selectedAutonLabel, ("Selected " + cppStr).c_str());
    }    
    return LV_RES_OK;
}
