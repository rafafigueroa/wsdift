#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""
import sys
from PyQt4 import QtGui

class Example(QtGui.QWidget):

    def __init__(self):
        super(Example,self).__init__()
        self.initUI()

    def initUI(self):

        okButton = QtGui.QPushButton("Ok")
        cancelButton = QtGui.QPushButton("Cancel")

        hbox = QtGui.QHBoxLayout()
        hbox.addStretch(1)
        hbox.addWidget(okButton)
        hbox.addWidget(cancelButton)

        vbox = QtGui.QVBoxLayout()
        vbox.addStretch(1)
        vbox.addLayout(hbox)

        self.setLayout(vbox)

        #main window
        self.setGeometry(300,300,250,150)
        self.setWindowTitle('Main Window')
        self.show()

def main():

    app = QtGui.QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()

