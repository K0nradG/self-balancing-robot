import sys
from PyQt6.QtWidgets import QApplication
from robot_control_app.robot_control_app import RobotControlApp


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotControlApp()
    window.show()
    sys.exit(app.exec())