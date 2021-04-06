import os
import glob

from hippocampus_common import radio_tools
import rospkg
from python_qt_binding import loadUi
from python_qt_binding import QtWidgets, QtCore


def scan_ports(prefix="ttyUSB"):
    path = os.path.join("/dev", prefix + "*")
    print(path)
    ports = glob.glob(path)
    ports.sort()
    return ports


def get_combo_option_by_text(combobox, text):
    for i in range(combobox.count()):
        if text == combobox.itemText(i):
            return i
    return -1


def enable_all_buttons_in_group(group, enabled):
    buttons = group.findChildren(QtWidgets.QPushButton)
    for button in buttons:
        button.setEnabled(enabled)


def show_error_box(message):
    error_diag = QtWidgets.QErrorMessage()
    error_diag.showMessage("{}".format(message))
    error_diag.exec_()


def show_message_box(message):
    message_diag = QtWidgets.QMessageBox()
    message_diag.setText("{}".format(message))
    message_diag.exec_()


class ComboBox(QtWidgets.QComboBox):
    popup = QtCore.pyqtSignal()

    def showPopup(self):
        self.popup.emit()
        super(ComboBox, self).showPopup()


class Worker(QtCore.QObject):
    ret = QtCore.pyqtSignal(object)
    finished = QtCore.pyqtSignal()

    def __init__(self, fun, *args, **kwargs):
        super(Worker, self).__init__()
        self.fun = fun
        self.args = args
        self.kwargs = kwargs

    def run(self):
        self.ret.emit(self.fun(*self.args, **self.kwargs))
        self.finished.emit()


class RadioConfiguratorWidget(QtWidgets.QWidget):
    BAUDS = [2400, 4800, 9600, 19200, 38400, 57600, 115200]
    BAUD_DEFAULT = 57600

    connected = QtCore.pyqtSignal()
    disconnected = QtCore.pyqtSignal()

    def __init__(self):
        super(RadioConfiguratorWidget, self).__init__()
        self.configurator = radio_tools.BaseConfigurator()
        self._is_connected = False
        self._in_at_mode = False

        self._enter_at_thread = None
        self._enter_at_worker = None

        self._show_version_thread = None
        self._show_version_worker = None

        self._read_eeprom_thread = None
        self._read_eeprom_worker = None
        self.setup_background_threads()
        ui_file = os.path.join(rospkg.RosPack().get_path("hippocampus_common"),
                               "resource", "radio_configurator.ui")

        loadUi(ui_file, self)
        self.connected.connect(self.on_connected)
        self.disconnected.connect(self.on_disconnected)
        self.init_connection_group()
        self.eeprom_group.setEnabled(False)
        self.init_eeprom_table()

    def setup_background_threads(self):
        def connect_worker_and_thread(worker, thread, return_cb, finished_cb):
            worker.moveToThread(thread)
            thread.started.connect(worker.run)
            worker.ret.connect(return_cb)
            worker.finished.connect(finished_cb)

        self._enter_at_thread = QtCore.QThread()
        self._enter_at_worker = Worker(self.configurator.enter_at_mode)
        connect_worker_and_thread(worker=self._enter_at_worker,
                                  thread=self._enter_at_thread,
                                  return_cb=self.on_enter_at_mode,
                                  finished_cb=self.on_enter_at_finished)

        self._show_version_thread = QtCore.QThread()
        self._show_version_worker = Worker(self.configurator.show_version)
        connect_worker_and_thread(worker=self._show_version_worker,
                                  thread=self._show_version_thread,
                                  return_cb=self.on_show_version,
                                  finished_cb=self.on_show_version_finished)

        self._read_eeprom_thread = QtCore.QThread()
        self._read_eeprom_worker = Worker(
            self.configurator.read_parameters_from_eeprom)
        connect_worker_and_thread(worker=self._read_eeprom_worker,
                                  thread=self._read_eeprom_thread,
                                  return_cb=self.on_read_eeprom,
                                  finished_cb=self.on_read_eeprom_finished)

    def on_enter_at_finished(self):
        self._enter_at_thread.quit()
        self._enter_at_thread.wait()

    def on_show_version_finished(self):
        self._show_version_thread.quit()
        self._show_version_thread.wait()

    def on_read_eeprom_finished(self):
        self._read_eeprom_thread.quit()
        self._read_eeprom_thread.wait()

    def get_table_column(self, name):
        col = -1
        name = name.lower()
        for i in range(self.eeprom_table.columnCount()):
            col_name = self.eeprom_table.horizontalHeaderItem(i).text().lower()
            if name == col_name:
                col = i
                break
        return col

    def get_table_row(self, param_name):
        col = -1
        try:
            col = self.configurator.EEPROM_PARAMETERS[
                param_name.lower()]["register"]
        except KeyError:
            pass
        return col

    def get_eeprom_current_column(self):
        return self.get_table_column("Current")

    def get_eeprom_desired_column(self):
        return self.get_table_column("Desired")

    def get_eeprom_default_column(self):
        return self.get_table_column("Default")

    def get_eeprom_param_name_column(self):
        return self.get_table_column("Param Name")

    def init_eeprom_table(self):
        params = self.configurator.EEPROM_PARAMETERS

        default_col = self.get_eeprom_default_column()
        param_name_col = self.get_eeprom_param_name_column()
        desired_col = self.get_eeprom_desired_column()

        for param in params:
            row = params[param]["register"]
            name = param.upper()
            options = params[param]["options"]
            default = params[param]["default"]
            if options:
                combo = QtWidgets.QComboBox()
                combo.addItems([str(option) for option in options])
                index = get_combo_option_by_text(combo, str(default))
                combo.setCurrentIndex(index)
                self.eeprom_table.setCellWidget(row, desired_col, combo)
            item = QtWidgets.QTableWidgetItem(name)
            self.eeprom_table.setItem(row, param_name_col, item)
            item = QtWidgets.QTableWidgetItem(str(default))
            self.eeprom_table.setItem(row, default_col, item)

    def init_connection_group(self):
        self.port_combobox.popup.connect(self.update_port_combobox)
        self.baud_combobox.addItems([str(baud) for baud in self.BAUDS])
        for i in range(len(self.BAUDS)):
            if str(self.BAUD_DEFAULT) == self.baud_combobox.itemText(i):
                self.baud_combobox.setCurrentIndex(i)

    def init_at_group(self):
        self.at_group.setEnabled(False)

    def update_port_combobox(self):
        print("updating port combobox")
        ports = scan_ports()
        print("available ports: {}".format(ports))
        current_text = self.port_combobox.currentText()
        self.port_combobox.clear()
        self.port_combobox.addItems(ports)
        if current_text in ports:
            for i in range(len(ports)):
                if current_text == self.port_combobox.itemText(i):
                    self.port_combobox.setCurrentIndex(i)
                    break

    @QtCore.pyqtSlot()
    def on_connected(self):
        self.connect_button.setEnabled(False)
        self._is_connected = True
        self.disconnect_button.setEnabled(True)

        self.eeprom_group.setEnabled(True)
        self.at_group.setEnabled(True)
        self.connection_status_label.setText("connected")

    @QtCore.pyqtSlot()
    def on_connect_button_clicked(self):
        port = self.port_combobox.currentText()
        baud = int(self.baud_combobox.currentText())
        print("Trying to open '{}@{}'".format(port, baud))
        self.configurator.port.port = port
        self.configurator.port.baudrate = baud
        try:
            if not self.configurator.port.is_open:
                self.configurator.port.open()
        except Exception as e:
            show_error_box(e)
            self.eeprom_group.setEnabled(False)
            self.connection_status_label.setText("failed to connect")
            self._is_connected = False
        else:
            self.connected.emit()

    @QtCore.pyqtSlot()
    def on_disconnected(self):
        self.disconnect_button.setEnabled(False)
        self.connect_button.setEnabled(True)
        self.connection_status_label.setText("disconnected")
        self.at_group.setEnabled(False)
        self.eeprom_group.setEnabled(False)

    @QtCore.pyqtSlot()
    def on_disconnect_button_clicked(self):
        self.configurator.exit_at_mode()
        self.configurator.port.flush()
        self.configurator.port.close()
        self.disconnected.emit()

    @QtCore.pyqtSlot()
    def on_enter_at_button_clicked(self):
        if self._enter_at_thread.isRunning():
            show_error_box("You are already trying to enter AT mode.")
        else:
            # self.enter_at_button.setEnabled(False)
            self._enter_at_thread.start()

    @QtCore.pyqtSlot(object)
    def on_enter_at_mode(self, success):
        if success:
            enable_all_buttons_in_group(self.at_group, True)
            self.enter_at_button.setEnabled(False)
            enable_all_buttons_in_group(self.eeprom_group, True)
        else:
            show_error_box(
                "Failed to enter AT mode. Make sure you are connected "
                "to the correct device and selected the appropriate baud rate.")
            enable_all_buttons_in_group(self.at_group, False)
            self.enter_at_button.setEnabled(True)
            enable_all_buttons_in_group(self.eeprom_group, False)

    @QtCore.pyqtSlot()
    def on_exit_at_button_clicked(self):
        self.configurator.exit_at_mode()
        enable_all_buttons_in_group(self.at_group, False)
        enable_all_buttons_in_group(self.eeprom_group, False)
        self.enter_at_button.setEnabled(True)

    @QtCore.pyqtSlot()
    def on_reboot_button_clicked(self):
        self.configurator.reboot()
        enable_all_buttons_in_group(self.at_group, False)
        enable_all_buttons_in_group(self.eeprom_group, False)
        self.enter_at_button.setEnabled(True)

    @QtCore.pyqtSlot()
    def on_show_version_button_clicked(self):
        if self._show_version_thread.isRunning():
            show_error_box("Already reading version...")
        else:
            self._show_version_thread.start()

    @QtCore.pyqtSlot(object)
    def on_show_version(self, version):
        version = str(version)
        if not version:
            show_error_box("Failed to get version!")
        else:
            show_message_box(version)

    @QtCore.pyqtSlot()
    def on_refresh_button_clicked(self):
        if self._read_eeprom_thread.isRunning():
            show_error_box("Already reading EEPROM...")
        else:
            self._read_eeprom_thread.start()

    @QtCore.pyqtSlot(object)
    def on_read_eeprom(self, eeprom):
        print("hello")
        if eeprom is None:
            print("error")
            show_error_box("Failed to read EEPROM!")
        else:
            column = self.get_eeprom_current_column()
            failed_names = []
            for param in eeprom:
                row = self.get_table_row(param)
                if row < 0:
                    failed_names.append(param)
                    continue
                print(row, column, param, eeprom[param])
                current_item = self.eeprom_table.item(row, column)
                if not current_item:
                    item = QtWidgets.QTableWidgetItem(str(eeprom[param]))
                    self.eeprom_table.setItem(row, column, item)
                else:
                    self.eeprom_table.item(row,
                                           column).setText(str(eeprom[param]))
            if len(failed_names) > 0:
                show_error_box(
                    "Failed to display following parameters: {}".format(
                        failed_names))
