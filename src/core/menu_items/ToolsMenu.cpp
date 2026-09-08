#include "ToolsMenu.h"
#include "core/display.h"
#include "core/utils.h"
#include "modules/gps/tcp_gps.h"
#include "modules/gps/tcp_wardriving.h"

void ToolsMenu::optionsMenu() {
    returnToMenu = false;
    options = {
        {"TCP GPS",            tcpGpsToolsMenu                     },
        {"TCP-GPS WarDriving", []() { TcpWardriving(true, true); } },
    };
    addOptionToMainMenu();
    loopOptions(options, MENU_TYPE_SUBMENU, "Tools");
    options.clear();
}

// Creates a weird ass icon, supposed to be a wrench
void ToolsMenu::drawIcon(float scale) {
    clearIconArea();
    int wrenchSize = scale * 55;
    int handleLen = scale * 30;
    int handleWidth = scale * 7;
    int centerX = iconCenterX;
    int centerY = iconCenterY;
    tft.drawRoundRect(
        centerX - handleLen / 2, centerY - handleWidth / 2, handleLen, handleWidth, 3, bruceConfig.priColor
    );
    tft.fillRect(centerX - handleLen / 2 - 12, centerY - 14, 20, 28, bruceConfig.priColor);
    tft.fillRect(centerX + handleLen / 2 - 8, centerY - 14, 20, 28, bruceConfig.priColor);
    tft.fillRect(centerX - 3, centerY - wrenchSize / 2, 6, wrenchSize, bruceConfig.priColor);
    tft.fillRect(centerX - wrenchSize / 2, centerY - 3, wrenchSize, 6, bruceConfig.priColor);
    for (int i = -2; i <= 2; i++) {
        tft.drawLine(
            centerX + i * 8,
            centerY - wrenchSize / 2,
            centerX + i * 8,
            centerY - wrenchSize / 2 - 8,
            bruceConfig.priColor
        );
        tft.drawLine(
            centerX + i * 8,
            centerY + wrenchSize / 2,
            centerX + i * 8,
            centerY + wrenchSize / 2 + 8,
            bruceConfig.priColor
        );
    }
}
