#!/usr/bin/env python

import sys
from collections import defaultdict
from random import randint
import time
import os

import pandas as pd
import numpy as np

import matplotlib
matplotlib.rcParams.update({'font.size': 8, 'scatter.marker': 'o',
    'lines.markersize': 1})

from matplotlib.backends.qt_compat import QtCore, QtWidgets, QtGui, is_pyqt5
if is_pyqt5():
    from matplotlib.backends.backend_qt5agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar)
else:
    from matplotlib.backends.backend_qt4agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar)
from matplotlib.figure import Figure


class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(ApplicationWindow, self).__init__()
        self._main = QtWidgets.QWidget()
        self.setCentralWidget(self._main)
        layout = QtWidgets.QGridLayout(self._main)
        splitter = QtWidgets.QSplitter()

        static_canvas = FigureCanvas(Figure(figsize=(5, 3)))
        splitter.addWidget(static_canvas)
        self.addToolBar(NavigationToolbar(static_canvas, self))

        self._static_ax = static_canvas.figure.subplots()

        chooser = QtWidgets.QPushButton('Choose Directory', self)
        chooser.clicked.connect(self.open_file_name_dialog)
        layout.addWidget(chooser, 1, 0)
        self.tree = QtWidgets.QTreeWidget()
        self.tree.itemClicked.connect(self.update_plot)
        self.tree.setSortingEnabled(True)

        plot_chooser = QtWidgets.QComboBox(self)
        plot_chooser.activated[str].connect(self.new_plot_type)


        self.plot_types = ("Scatter Plot", "Line Plot")
        self.plot_type = self.plot_types[0]
        plot_chooser.addItems(self.plot_types)

        fit_x = QtWidgets.QPushButton('Fit X', self)
        fit_x.clicked.connect(self.fit_x)
        fit_y = QtWidgets.QPushButton('Fit Y', self)
        fit_y.clicked.connect(self.fit_y)
        fit_match = QtWidgets.QPushButton('Fit Match', self)
        fit_match.clicked.connect(self.fit_match)
        splitter.addWidget(self.tree)
        layout.addWidget(splitter, 0, 0, 1, 5)
        layout.addWidget(plot_chooser, 1, 1)
        layout.addWidget(fit_x, 1, 2)
        layout.addWidget(fit_y, 1, 3)
        layout.addWidget(fit_match, 1, 4)
        self.autoscale = True

        if (sys.argv[1] and os.path.exists(sys.argv[1])):
            self.load_dir(sys.argv[1])

    def new_plot_type(self, choice):
        self.plot_type = choice
        self.update_plot()

    def fit_y(self):
        xlim = (self._static_ax.get_xlim())
        lower = 5e10
        upper = -5e10
        for f,c in self.plotted_items:
            df = self.dataframes[f]
            x = np.array(df.timestamp)

            for y in c:
                y_sel = np.array(df[y])
                y_sel = y_sel[np.logical_and(x > xlim[0], x < xlim[1])]

                lower = min(lower, y_sel.min())
                upper = max(upper, y_sel.max())

        self._static_ax.set_ylim((lower + (lower - upper) * 0.05, upper + (upper - lower) * 0.05))
        self._static_ax.figure.canvas.draw()


    def fit_x(self):
        x = np.array([])
        lower = 5e10
        upper = -5e10

        for f,c in self.plotted_items:
            df = self.dataframes[f]
            x = np.array(df.timestamp)
            lower = min(lower, x.min())
            upper = max(upper, x.max())

        self._static_ax.set_xlim((lower, upper))
        self._static_ax.figure.canvas.draw()

    def fit_match(self):
        key = 'driver_station_status.csv'
        if not (key in self.dataframes):
            print("ERR: Tried to fit auto without DS Status")
            return

        fit_types = ("Auto Period", "Teleop Period", "Full Match Period")

        fit_type, ok = QtWidgets.QInputDialog.getItem(
            self, "Fit Match", "Match Data Type:",
            fit_types, 0, False)

        df = self.dataframes[key]
        x = np.array(df.timestamp)
        mode = np.array(df['mode'])

        if not (ok and fit_type):
            print("ERR: Didn't select fit type")
            return

        if (fit_type == fit_types[0]):
            idx = x[mode == 1]
        elif (fit_type == fit_types[1]):
            idx = x[mode == 2]
        elif (fit_type == fit_types[2]):
            idx = x[mode.nonzero()]
        else:
            print("ERR: Unexpected Fit Type")

        print ("INFO: Fit to " + fit_type)

        self._static_ax.set_xlim((idx.min(), idx.max()))
        self._static_ax.figure.canvas.draw()
        self.autoscale = False

    def open_file_name_dialog(self):
        directory = QtWidgets.QFileDialog.getExistingDirectory(self, 'Select directory')
        files = (os.listdir(directory))
        self.csvs = []
        self.dataframes = {}
        for s in files:
            if s.endswith("csv"):
                self.csvs.append(s)

        for c in self.csvs:
            self.dataframes[c] = (pd.read_csv(directory + "/" + c))
            self.update_side_bar(self.dataframes[c].columns.tolist(), c)
            self._static_ax.clear()
            self.plotted_items = []


    def update_side_bar(self, columns, filename):
        parent = QtWidgets.QTreeWidgetItem(self.tree)
        parent.setFlags(parent.flags() | QtCore.Qt.ItemIsTristate)
        parent.setText(0, filename)
        for c in columns:
            child = QtWidgets.QTreeWidgetItem(parent)
            child.setFlags(child.flags() | QtCore.Qt.ItemIsUserCheckable)
            child.setText(0, c)
            child.setCheckState(0, QtCore.Qt.Unchecked)

    def update_plot(self):
        xlim = self._static_ax.get_xlim()
        ylim = self._static_ax.get_ylim()

        self._static_ax.clear()
        to_plot = self.export_tree()

        for fname, colnames in to_plot.items():
            df = self.dataframes[fname]
            for colname in colnames:
                if colname == "timestamp":
                    continue
                if not colname:
                    continue
                if self.plot_type == self.plot_types[0]:
                    self._static_ax.scatter(df.timestamp, df[colname], label=colname)
                elif self.plot_type == self.plot_types[1]:
                    self._static_ax.plot(df.timestamp, df[colname], label=colname)
                else:
                    print("ERR: Unknown plot type, " + self.plot_type)

                print("INFO: Plotting " + fname + " " + colname)

        if not self.autoscale:
            self._static_ax.set_xlim(xlim)
            self._static_ax.set_ylim(ylim)

        self.plotted_items = to_plot.items()

        self._static_ax.legend(loc='upper right')
        self._static_ax.figure.canvas.draw()

        self.autoscale = False

    def export_tree(self):
        mapping = defaultdict(list)
        root = self.tree.invisibleRootItem()
        for index in range(root.childCount()):
            parent = root.child(index)
            if parent.checkState(0) == QtCore.Qt.PartiallyChecked or parent.checkState(0) == QtCore.Qt.Checked:
                features = mapping[parent.text(0)]
                for row in range(parent.childCount()):
                    child = parent.child(row)
                    if child.checkState(0) == QtCore.Qt.Checked and child.text(0) != "timestamp" and child.text(0) != "queue_index":
                        features.append(child.text(0))
        return mapping

if __name__ == "__main__":
    qapp = QtWidgets.QApplication(sys.argv)
    app = ApplicationWindow()
    app.show()
    qapp.exec_()
