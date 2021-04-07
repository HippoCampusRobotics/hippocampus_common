import os
import glob
import logging

from hippocampus_common import radio_tools
import rospkg
from python_qt_binding import loadUi
from python_qt_binding import QtWidgets, QtCore

logger = logging.getLogger("radio_gui")


def scan_ports(prefix="ttyUSB"):
    path = os.path.join("/dev", prefix + "*")
    ports = glob.glob(path)
    ports.sort()
    logger.debug("Available ports: %s", ports)
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


class QTextEditLogger(logging.Handler):
    def __init__(self, textedit_widget):
        super(QTextEditLogger, self).__init__()
        self.widget = textedit_widget
        self.widget.setReadOnly(True)

    def emit(self, record):
        msg = self.format(record)
        self.widget.append(msg)


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
        self.logging_handler = QTextEditLogger(self.console_text)
        self.logging_handler.setFormatter(
            logging.Formatter("%(asctime)s - %(levelname)s - %(message)s"))
        self.logger = logging.getLogger("radio_gui")
        self.logger.addHandler(self.logging_handler)
        self.logger.setLevel(logging.DEBUG)
        self.init_logging_radiobuttons()
        self.connected.connect(self.on_connected)
        self.disconnected.connect(self.on_disconnected)
        self.init_connection_group()
        self.eeprom_group.setEnabled(False)
        self.init_eeprom_table()
        self.init_splitter()

    def init_splitter(self):
        handle = self.splitter.handle(1)
        layout = QtWidgets.QVBoxLayout(handle)
        # layout.setSpacing(0)
        line = QtWidgets.QFrame(handle)
        line.setFrameShape(QtWidgets.QFrame.HLine)
        line.setFrameShadow(QtWidgets.QFrame.Sunken)
        layout.addWidget(line)

    def init_logging_radiobuttons(self):
        def connect_button_toggled(button):
            button.toggled.connect(
                lambda: self.handle_logging_radiobutton(button))

        buttons = self.logger_level_group.findChildren(QtWidgets.QRadioButton)
        for button in buttons:
            connect_button_toggled(button)
        self.info_radiobutton.setChecked(True)

    def handle_logging_radiobutton(self, button):
        if not button.isChecked():
            return
        if button.text() == "Debug":
            self.logger.setLevel(logging.DEBUG)
            self.logger.debug("Setting debug level to %s.",
                              button.text().upper())
        elif button.text() == "Info":
            self.logger.debug("Setting debug level to %s.",
                              button.text().upper())
            self.logger.setLevel(logging.INFO)
        elif button.text() == "Warn":
            self.logger.debug("Setting debug level to %s.",
                              button.text().upper())
            self.logger.setLevel(logging.WARN)
        elif button.text() == "Error":
            self.logger.debug("Setting debug level to %s.",
                              button.text().upper())
            self.logger.setLevel(logging.ERROR)
        else:
            self.logger.error("Unhandled logging level: %s.", button.text())

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
        ports = scan_ports()
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
        self.logger.info("Connected to '%s@%s'", self.configurator.port.port,
                         self.configurator.port.baudrate)
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
            self.logger.info("Entering AT mode...")
            self._enter_at_thread.start()

    @QtCore.pyqtSlot(object)
    def on_enter_at_mode(self, success):
        if success:
            self.logger.info("Entered AT mode successfully.")
            enable_all_buttons_in_group(self.at_group, True)
            self.enter_at_button.setEnabled(False)
            enable_all_buttons_in_group(self.eeprom_group, True)
            self.do_refresh()
        else:
            self.logger.error("Failed to enter AT mode.")
            show_error_box(
                "Failed to enter AT mode. Make sure you are connected "
                "to the correct device and selected the appropriate baud rate.")
            enable_all_buttons_in_group(self.at_group, False)
            self.enter_at_button.setEnabled(True)
            enable_all_buttons_in_group(self.eeprom_group, False)

    @QtCore.pyqtSlot()
    def on_exit_at_button_clicked(self):
        self.configurator.exit_at_mode()
        self.logger.info("Exited AT mode.")
        enable_all_buttons_in_group(self.at_group, False)
        enable_all_buttons_in_group(self.eeprom_group, False)
        self.enter_at_button.setEnabled(True)

    @QtCore.pyqtSlot()
    def on_reboot_button_clicked(self):
        self.configurator.reboot()
        self.logger.info("Rebooted.")
        enable_all_buttons_in_group(self.at_group, False)
        enable_all_buttons_in_group(self.eeprom_group, False)
        self.enter_at_button.setEnabled(True)

    @QtCore.pyqtSlot()
    def on_show_version_button_clicked(self):
        if self._show_version_thread.isRunning():
            show_error_box("Already reading version...")
            self.logger.warn(
                "Trying to read firmware version, but the thread is already "
                "running.")
        else:
            self.logger.warn("Reading firmware version...")
            self._show_version_thread.start()

    @QtCore.pyqtSlot(object)
    def on_show_version(self, version):
        version = str(version)
        if not version:
            self.logger.error("Failed to read firmware version.")
            show_error_box("Failed to get version!")
        else:
            self.logger.info("Firmware version: %s", version)
            show_message_box(version)

    @QtCore.pyqtSlot()
    def on_refresh_button_clicked(self):
        self.logger.debug("Refresh button clicked.")
        self.do_refresh()

    def do_refresh(self):
        if self._read_eeprom_thread.isRunning():
            self.logger.warn("Trying to read EEPROM, but the responsible "
                             "thread is already running.")
            show_error_box("Already reading EEPROM...")
        else:
            self.logger.info("Refreshing current EEPROM parameters.")
            self._read_eeprom_thread.start()

    @QtCore.pyqtSlot(object)
    def on_read_eeprom(self, eeprom):
        if eeprom is None:
            self.logger.error("Failed to read EEPROM parameters!")
            show_error_box("Failed to read EEPROM!")
        else:
            column = self.get_eeprom_current_column()
            failed_names = []
            for param in eeprom:
                row = self.get_table_row(param)
                if row < 0:
                    failed_names.append(param)
                    continue
                current_item = self.eeprom_table.item(row, column)
                if not current_item:
                    item = QtWidgets.QTableWidgetItem(str(eeprom[param]))
                    self.eeprom_table.setItem(row, column, item)
                else:
                    self.eeprom_table.item(row,
                                           column).setText(str(eeprom[param]))
            self.logger.info("Refreshed EEPROM paramters successfully.")
            if len(failed_names) > 0:
                self.logger.error("Failed to display following paramters: %s",
                                  failed_names)
                show_error_box(
                    "Failed to display following parameters: {}".format(
                        failed_names))

    def set_desired_cell(self, row, value):
        col = self.get_eeprom_desired_column()
        if col < 0:
            return False
        value = str(value)
        widget = self.eeprom_table.cellWidget(row, col)
        if widget is None:
            item = QtWidgets.QTableWidgetItem(value)
            self.eeprom_table.setItem(row, col, item)
        elif isinstance(widget, QtWidgets.QComboBox):
            index = get_combo_option_by_text(widget, value)
            if index < 0:
                return False
            widget.setCurrentIndex(index)
        else:
            item = self.eeprom_table.item(row, col)
            item.setText(value)
        return True

    def copy_column_to_desired(self, col):
        failed = []
        for row in range(self.eeprom_table.rowCount()):
            try:
                value = self.eeprom_table.item(row, col).text()
            except AttributeError:
                failed.append(row)
            else:
                success = self.set_desired_cell(row, value)
                if not success:
                    failed.append(row)
        if len(failed) > 0:
            show_error_box(
                "Failed to read or set the following rows: {}".format(failed))

    @QtCore.pyqtSlot()
    def on_set_default_button_clicked(self):
        self.logger.debug("Set default button clicked.")
        col = self.get_eeprom_default_column()
        self.copy_column_to_desired(col)

    @QtCore.pyqtSlot()
    def on_revert_button_clicked(self):
        self.logger.debug("Revert button clicked.")
        col = self.get_eeprom_current_column()
        self.copy_column_to_desired(col)

    @QtCore.pyqtSlot()
    def on_apply_button_clicked(self):
        params = radio_tools.BaseConfigurator.get_default_params()
        desired_col = self.get_eeprom_desired_column()
        current_col = self.get_eeprom_current_column()
        name_col = self.get_eeprom_param_name_column()
        unset_params = []
        unknown_params = []
        changed_params = {}
        for row in range(self.eeprom_table.rowCount()):
            name = self.eeprom_table.item(row, name_col).text().lower()
            if name not in params:
                unknown_params.append(name)
                continue
            widget = self.eeprom_table.cellWidget(row, desired_col)
            if isinstance(widget, QtWidgets.QComboBox):
                value = widget.currentText()
            else:
                try:
                    value = self.eeprom_table.item(row, desired_col).text()
                except AttributeError:
                    unset_params.append(name)
                    continue
            params[name] = value
            current_value = self.eeprom_table.item(row, current_col).text()
            if value != current_value:
                changed_params[name] = dict(old_val=current_value,
                                            new_val=value)
        text = "Chaning following parameters:\n\n"
        for param_name in changed_params:
            text += "{}: {} -> {}\n".format(
                param_name, changed_params[param_name]["old_val"],
                changed_params[param_name]["new_val"])
        text += "\nDo you want to continue?"
        reply = QtWidgets.QMessageBox.question(self, "Apply Changes", text,
                                               QtWidgets.QMessageBox.Yes,
                                               QtWidgets.QMessageBox.No)
        if reply == QtWidgets.QMessageBox.No:
            self.logger.info("Writing of parameters canceled.")
            return
        self.configurator.enter_at_mode()
        self.configurator.set_desired_params(params)
        self.logger.info("Writing paramters...")
        failed_params = self.configurator.write_params()
        if len(failed_params) > 0:
            show_error_box("Failed to write params: {}".format(failed_params))
        self.do_refresh()
        if len(unset_params):
            show_message_box(
                "Ignored unset parameters: {}".format(unset_params))
        if len(unknown_params):
            show_error_box("Unknown parameters could not be written: {}".format(
                unknown_params))

    @QtCore.pyqtSlot()
    def on_make_persistent_button_clicked(self):
        reply = QtWidgets.QMessageBox.question(
            self, "Writing EEPROM",
            "Writing EEPROM is required to make changes persistent. Keep in mind, that the number of EEPROM writes is limited.\nContinue?",
            QtWidgets.QMessageBox.Yes, QtWidgets.QMessageBox.No)
        if reply == QtWidgets.QMessageBox.Yes:
            self.on_apply_button_clicked()
            self.configurator.write_eeprom()
            show_message_box(
                "EEPROM has been written :-)\n\nSome parameter changes may require a reboot to take effect."
            )
