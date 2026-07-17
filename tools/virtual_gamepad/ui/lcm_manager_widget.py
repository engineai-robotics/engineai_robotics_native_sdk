from PyQt6.QtWidgets import (QWidget, QHBoxLayout, QPushButton, QLabel,
                             QMessageBox, QComboBox)
from core.lcm_manager import LcmManager


class LcmManagerWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.lcm_manager = LcmManager()
        self.init_ui()
        self.init_ui_signal()
        self.lcm_manager.add_connection_listener(self.on_lcm_status_changed)
        self.on_lcm_status_changed(self.lcm_manager.is_connected())

    def init_ui(self):
        layout = QHBoxLayout()
        self.setLayout(layout)

        self.url_label = QLabel("LCM URL:")
        self.url_combo = QComboBox()
        self.url_combo.setEditable(True)
        self.url_combo.addItem("udpm://239.255.76.67:7667?ttl=0")
        self.url_combo.addItem(
            "udpm://239.255.76.67:7667?ttl=1&interface=eth0&recv_buf_size=2097152")
        self.url_combo.addItem("udpm://239.255.76.77:7667?ttl=10")
        self.url_combo.setCurrentText("udpm://239.255.76.67:7667?ttl=0")
        self.url_combo.setPlaceholderText("Enter LCM URL")

        self.connect_button = QPushButton("Connect")
        self.status_label = QLabel("Disconnected")

        layout.addWidget(self.url_label)
        layout.addWidget(self.url_combo, stretch=1)
        layout.addWidget(self.connect_button)
        layout.addWidget(self.status_label)

    def init_ui_signal(self):
        self.connect_button.clicked.connect(self.toggle_connection)

    def toggle_connection(self):
        if self.lcm_manager.is_connected():
            self.lcm_manager.safe_disconnect()
            return

        success, message = self.lcm_manager.connect(self.url_combo.currentText())
        if not success:
            QMessageBox.warning(self, "LCM Connection", message)

    def on_lcm_status_changed(self, connected):
        self.status_label.setText("Connected" if connected else "Disconnected")
        self.connect_button.setText("Disconnect" if connected else "Connect")

    def closeEvent(self, event):
        self.lcm_manager.remove_connection_listener(self.on_lcm_status_changed)
        super().closeEvent(event)
