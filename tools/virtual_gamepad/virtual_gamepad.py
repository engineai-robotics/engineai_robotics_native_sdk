import os
import signal
import sys

from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QIcon
from PyQt6.QtWidgets import QApplication, QMainWindow, QTabWidget, QVBoxLayout, QWidget
from ui.lcm_manager_widget import LcmManagerWidget
from ui.virtual_gamepad_widget import VirtualGamepadWidget


class VirtualGamepadWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Virtual Gamepad")

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Virtual Gamepad")
        current_directory = os.path.dirname(os.path.abspath(__file__))
        self.setWindowIcon(
            QIcon(os.path.join(current_directory, 'assets/logo.jpg')))

        self.lcm_manager = LcmManagerWidget()
        self.main_layout.addWidget(self.lcm_manager)

        self.tab_widget = QTabWidget()
        self.tab_widget.setTabPosition(QTabWidget.TabPosition.North)

        self.gamepad_simulator = VirtualGamepadWidget()
        self.tab_widget.addTab(self.gamepad_simulator, "Virtual Gamepad")

        self.main_layout.addWidget(self.tab_widget)


def signal_handler(signal, frame):
    print("Ctrl+C detected! Exiting...")
    QApplication.quit()


if __name__ == "__main__":
    # Register the signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    app = QApplication(sys.argv)
    window = VirtualGamepadWindow()
    window.show()
    if sys.platform != "win32":
        signal.siginterrupt(signal.SIGINT, False)

    # Use QTimer to periodically check for signals
    timer = QTimer()
    # Do nothing, just to allow the event loop to handle signals
    timer.timeout.connect(lambda: None)
    timer.start(100)
    sys.exit(app.exec())
