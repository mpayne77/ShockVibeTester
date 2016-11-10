# I'm setting the Kivy directory to be a sub of the application directory
# I ran into some problems with system path variables on my desktop box where
# the Kivy defaults were conflicting with our Creo setup. This likely isn't
# necessary on the 'production' setup, but I don't see any harm in leaving it
# here
import os
working_dir = os.getcwd()
kivy_dir = working_dir + '/.kivy'
os.environ['KIVY_HOME'] = kivy_dir

# Kivy imports
import kivy
from kivy.app import App
from kivy.core.window import Window
from kivy.properties import NumericProperty, StringProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.checkbox import CheckBox
from kivy.clock import Clock
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.image import Image
from kivy.uix.label import Label
from kivy.uix.popup import Popup
from kivy.uix.textinput import TextInput
from kivy.uix.widget import Widget

# Config imports
from config import COM_PORT, DEV_MODE, SCREEN_RES

# Other imports
import numpy as np
import queue
import serial
import struct
import threading

from math import floor
from time import sleep

################################################################################
###### Configuration variables #################################################
################################################################################
""" These should not be edited. User settable configuration values are in the
    file config.py
"""
# Set serial com config for installation. For a windows computer you should
# not need to change any value other than 'PORT'
COM_CONFIG = { 'SAMPLESIZE': 17,
               'FRAMESIZE': 68,
               'PORT': COM_PORT,
               'BAUDRATE': 115200 }

# This is the time interval in seconds between attempted USB reads. This value
# needs to be set shorter than the corresponding write interval on the
# Arduino, otherwise the elapsed time clock will get out of sync and the UI
# will noticeably lag behind the sensor outputs.
USB_READ_INTERVAL = 0.05

DATAFRAME = np.zeros(COM_CONFIG['SAMPLESIZE'], dtype=np.uint32)
"""The DATAFRAME is a 1x<SAMPLESIZE> array of unsigned 32-bit integers.
   The contents are as follows:

   Index    Contents
   0        Running elapsed time in milliseconds
   1        Event counter - discrete channel 1
   2        Event counter - discrete channel 2
   3        Event counter - discrete channel 3
   4        Event counter - discrete channel 4
   5        Event counter - discrete channel 5
   6        Event counter - discrete channel 6
   7        Event counter - discrete channel 7
   8        Event counter - discrete channel 8
   9        Event counter - analog channel 1
   10       Current value - analog channel 1
   11       Event counter - analog channel 2
   12       Current value - analog channel 2
   13       Event counter - analog channel 3
   14       Current value - analog channel 3
   15       Event counter - analog channel 4
   16       Current value - analog channel 4
"""
################################################################################
##### End configuration variables ##############################################
################################################################################


class AnalogLabels(BoxLayout):
    pass


class AnalogLayout(BoxLayout):
    pass


class AnalogMeter(FloatLayout):
    channelID = NumericProperty()

    # Cosmetic margin used in GUI layout
    margin = NumericProperty(4)

    # Threshold values used by Arduino (0 to 1023)
    lowThreshold = NumericProperty(0)
    highThreshold = NumericProperty(1023)

    # Threshold values for display (0-10V or 0-20mA)
    lowThresholdDisp = StringProperty('Low Thres.\r\n------------')
    highThresholdDisp = StringProperty('High Thres.\r\n----------')

    # Analog level sent back from Arduino (0 to 1023)
    level = NumericProperty(0)

    def on_touch_move(self, touch):
        '''Moves the threshold markers'''

        # Do not move markers if test is running
        if app.controlsLayout.testRunning == True:
            return None

        # Only act on touches within AnalogMeter
        if self.collide_point(*touch.pos):

            # Ignore touch if the channel is deactivated
            if app.analogStrips[self.channelID-1].ids.channelLabel.state == 'normal':
                return None

            # Only act further if the touch position is within one of the
            # two threshold markers
            if self.ids.markerLow.collide_point(*touch.pos) | \
                self.ids.markerHigh.collide_point(*touch.pos):
                xBase = self.ids.bgMeter.x
                xRight = self.ids.bgMeter.right

                # Move only the marker closest to the touch position
                highDist = abs(touch.x - self.ids.markerHigh.center_x)
                lowDist = abs(touch.x - self.ids.markerLow.center_x)
                if highDist <= lowDist:
                    # Don't let marker move off of the end of the indicator
                    if touch.x > (xRight - self.margin):
                        self.ids.markerHigh.center_x = xRight - self.margin
                    else:
                        self.ids.markerHigh.center_x = touch.x
                    # Set threshold (0 to 1023) value according to marker position
                    self.highThreshold = int((self.ids.markerHigh.center_x-(xBase+self.margin))*1023/(xRight-xBase-2*self.margin))
                else:
                    if touch.x < (self.ids.bgMeter.x + self.margin):
                        self.ids.markerLow.center_x = self.ids.bgMeter.x + self.margin
                    else:
                        self.ids.markerLow.center_x = touch.x
                    self.lowThreshold = int((self.ids.markerLow.center_x-(xBase+self.margin))*1023/(xRight-xBase-2*self.margin))

                # Update channel configuration
                app.analogStrips[self.channelID-1].updateConfig()

    def updateLabels(self):
        '''Updates the labels showing the threshold values'''
        if app.analogStrips[self.channelID-1].ids.channelLabel.state == 'normal':
            self.lowThresholdDisp = 'Low Thres.\r\n------------'
            self.highThresholdDisp = 'High Thres.\r\n----------'

        elif app.analogStrips[self.channelID-1].ids.currentButton.state == 'down':
            mA = self.highThreshold * 20/1023.0
            mAdisp = '{:0.2f} mA'.format(mA)
            self.highThresholdDisp = 'High Thres.\r\n' + mAdisp

            mA = self.lowThreshold * 20/1023.0
            mAdisp = '{:0.2f} mA'.format(mA)
            self.lowThresholdDisp = 'High Thres.\r\n' + mAdisp

        else:
            volts = self.highThreshold * 10/1023.0
            voltsDisp = '{:0.3f} V'.format(volts)
            self.highThresholdDisp = 'High Thres.\r\n' + voltsDisp

            volts = self.lowThreshold * 10/1023.0
            voltsDisp = '{:0.3f} V'.format(volts)
            self.lowThresholdDisp = 'High Thres.\r\n' + voltsDisp


class ChannelStripAnalog(BoxLayout):
    channelID = NumericProperty()

    # Running tally of threshold crossing events received from Arduino
    eventCounter = NumericProperty(0)

    # Channel configuration value:
    #   0: Off
    #   1: Analog current (0-20mA)
    #   2: Analog voltage (0-10V)
    # High and low threshold values belong to the AnalogMeter class
    channelConfig = NumericProperty(0)

    def updateConfig(self):
        '''Updates the channel configuration'''

        # If test is not running, update channel configuration according to
        # GUI state
        if app.controlsLayout.testRunning == False:
            self.ids.analogMeter.updateLabels()
            if self.ids.channelLabel.state == 'normal':
                print('Channel A{} is off'.format(str(self.channelID)))
                self.channelConfig = 0
            elif self.ids.currentButton.state == 'down':
                print('Channel A{} is analog current, low threshold {}, high threshold {}'.format(
                str(self.channelID), str(self.ids.analogMeter.lowThreshold),
                str(self.ids.analogMeter.highThreshold)))
                self.channelConfig = 1
            else:
                print('Channel A{} is analog voltage, low threshold {}, high threshold {}'.format(
                str(self.channelID), str(self.ids.analogMeter.lowThreshold),
                str(self.ids.analogMeter.highThreshold)))
                self.channelConfig = 2

        # If test is running. Immediately return GUI to state in accordance with
        # existing config values. This effectively freezes the GUI elements while
        # a test is running (not sure if there's a more straightforward way
        # of doing this).
        else:
            if self.channelConfig == 0:
                self.ids.channelLabel.state = 'normal'
            if self.channelConfig == 1 or self.channelConfig == 2:
                self.ids.channelLabel.state = 'down'
                if self.channelConfig == 1:
                    self.ids.currentButton.state = 'down'
                    self.ids.voltageButton.state = 'normal'
                else:
                    self.ids.currentButton.state = 'normal'
                    self.ids.voltageButton.state = 'down'


class ChannelConfigButton(CheckBox):
    pass


class ChannelStripDiscrete(BoxLayout):
    channelID = NumericProperty()

    # Running tally of threshold crossing events received from Arduin
    eventCounter = NumericProperty(0)

    # Channel configuration value:
    #   0: Off
    #   1: PNP/NPN rising edge
    #   2: PNP/NPN falling edge
    channelConfig = NumericProperty(0)

    def updateConfig(self):
        """Updates the channel configuration."""

        # If test is not running, update channel configuration according to
        # GUI state
        if app.controlsLayout.testRunning == False:
            if self.ids.channelLabel.state == 'normal':
                print('Channel D{} is off'.format(str(self.channelID)))
                self.channelConfig = 0
            else:
                if self.ids.risingEdgeButton.state == 'down':
                    print('Channel D{} is PNP/NPN rising edge'.format(str(self.channelID)))
                    self.channelConfig = 1
                else:
                    print('Channel D{} is PNP/NPN falling edge'.format(str(self.channelID)))
                    self.channelConfig = 2

        # If test is running. Immediately return GUI to state in accordance with
        # existing config values. This effectively freezes the GUI elements while
        # a test is running.
        else:
            if self.channelConfig == 0:
                self.ids.channelLabel.state = 'normal'
            if self.channelConfig == 1 or self.channelConfig == 2:
                self.ids.channelLabel.state = 'down'
                if self.channelConfig == 1:
                    self.ids.risingEdgeButton.state = 'down'
                    self.ids.fallingEdgeButton.state = 'normal'
                else:
                    self.ids.risingEdgeButton.state = 'normal'
                    self.ids.fallingEdgeButton.state = 'down'


class CheckBoxLabel(Label):
    pass


class ControlsLayout(BoxLayout):
    testRunning = False

    def readData(self, dt):
        """Reads and processes data from serial queue.

           Queue is first in first out. In practice, since we are calling this
           function at a higher frequency than we are sending serial packets
           from the Arduino, the queue will have either 0 or 1 item in it."""

        # The queue is usually empty
        if self.q.empty():
            return None

        # When the queue is not empty, get the first packet of data off of the
        # queue and process.
        dataframe = self.q.get()
        self.ids.elapsedTime.updateClock(dataframe[0])

        for strip in app.discreteStrips:
            strip.eventCounter = int(dataframe[strip.channelID])

        for strip in app.analogStrips:
            strip.ids.analogMeter.level = int(dataframe[(strip.channelID*2)+8])
            strip.eventCounter = int(dataframe[(strip.channelID*2)+7])

    def startTest(self):
        """Starts a test"""
        if self.testRunning:
            return None

        try:
            self.usb = serial.Serial(COM_CONFIG['PORT'], COM_CONFIG['BAUDRATE'],
                                timeout = 10)
            sleep(0.5)

            for strip in app.discreteStrips:
                self.usb.write(struct.pack('<B', strip.channelConfig))

            for strip in app.analogStrips:
                self.usb.write(struct.pack('<B', strip.channelConfig))
                self.usb.write(struct.pack('<H', strip.ids.analogMeter.lowThreshold))
                self.usb.write(struct.pack('<H', strip.ids.analogMeter.highThreshold))

            self.usb.write(struct.pack('<L', self.ids.gateThreshold.gateThresholdMicros))

            self.ids.startButton.disabled = True
            self.ids.stopButton.disabled = False

        except serial.serialutil.SerialException:
            print('Serial connection could not be established. Program exiting.')
            raise SystemExit

        #sleep(1)
        self.usb.reset_input_buffer()
        self.q = queue.Queue()
        self.reader = SerialReader(self.q, self.usb)
        self.reader.start()
        self.testRunning = True

        # The data read should always be scheduled for a shorter interval than
        # the arduino is sending updates. Otherwise the UI display will lag
        # behind what is happening in real time.
        Clock.schedule_interval(self.readData, USB_READ_INTERVAL)

    def stopTest(self):
        """Stops a running test."""
        Clock.unschedule(self.readData)
        if self.testRunning:
            self.reader.stop()
            self.ids.startButton.disabled = False
            self.ids.stopButton.disabled = True
            self.testRunning = False
        else:
            pass


class DiscreteLabels(BoxLayout):
    pass


class DiscreteLayout(BoxLayout):
    pass


class EventCounter(Label):
    pass


class ElapsedTime(BoxLayout):
    timeDisplay = StringProperty('--:--:--')

    def updateClock(self, timeMillis):
        """Update the displayed clock value.

           Takes elapsed time in milliseconds, converts to HH:MM:SS format
           and updates GUI display."""

        seconds = timeMillis/1000.0
        hours = floor(seconds/3600)
        minutes = floor(seconds/60 - hours*60)
        seconds = floor(seconds - hours*3600 - minutes*60)
        self.timeDisplay = '{:02d}:{:02d}:{:02d}'.format(hours, minutes, seconds)


class GateThresholdControls(BoxLayout):
    # Gate threshold in milliseconds
    gateThresholdMillis = NumericProperty(1.0)

    # Gate threshold in microseconds
    gateThresholdMicros = NumericProperty(1000)

    # Gate treshold for GUI display
    gateThresholdStr = StringProperty('1.00 ms')

    def decrementThreshold(self):
        """Increments gate threshold.

           The increment/decrement value is currently hard-coded to 0.25ms.

           Future improvement: Change increment/decrement value according to
           or smaller, the increment/decrement should as well). Or possibly
           current threshold value (i.e. as the threshold value gets larger
           variable speed scrolling as the buttons are held down."""

        if app.controlsLayout.testRunning == True:
            return None

        if self.gateThresholdMillis >= 0.25:
            self.gateThresholdMillis -= 0.25
            self.gateThresholdStr = '{:0.2f} ms'.format(self.gateThresholdMillis)
            self.gateThresholdMicros = int(self.gateThresholdMillis*1000)

    def incrementThreshold(self):
        """Decrements gate threshold"""

        if app.controlsLayout.testRunning == True:
            return None

        self.gateThresholdMillis += 0.25
        self.gateThresholdStr = '{:0.2f} ms'.format(self.gateThresholdMillis)
        self.gateThresholdMicros = int(self.gateThresholdMillis*1000)

    def manualEntry(self):
        ''' Allows manual entry of gate threshold value '''

        if app.controlsLayout.testRunning == True:
            return None

        popup = Popup(title = 'Enter detection threshold time in ms',
        content=TextInput(multiline=False), size=(500, 100), size_hint=(None, None))
        popup.bind(on_dismiss=self.setThreshold)
        popup.open()

    def setThreshold(self, popup):
        try:
            self.gateThresholdMillis = float(popup.content.text)
        except ValueError:
            return None

        self.gateThresholdMicros = int(self.gateThresholdMillis*1000)
        self.gateThresholdStr = '{:0.2f} ms'.format(self.gateThresholdMillis)


class LineSeparator(Widget):
    pass


class PassFailCheckBox(CheckBox):
    pass


class SerialReader(threading.Thread):
    def __init__(self, q, usb):
        threading.Thread.__init__(self)
        self.q = q
        self.usb = usb
        self.stopFlag = False
        self.daemon = True

    def serialRead(self):
        """Reads one packet of incoming serial data"""

        data = self.usb.read(COM_CONFIG['FRAMESIZE'])
        DATAFRAME = np.fromstring(data, dtype=np.uint32)
        return DATAFRAME

    def run(self):
        while not self.stopFlag:
            self.q.put(self.serialRead())
        print('Reader thread stopping')

    def stop(self):
        self.stopFlag = True
        self.usb.close()


class ThresholdMarker(Image):
    pass


Window.size = SCREEN_RES
if DEV_MODE == False:
    Window.fullscreen = True

class ShockVibeBoxApp(App):
    def build(self):
        # Top level layout is 3 panel side-by-side box
        mainLayout = BoxLayout(orientation='horizontal')
        discreteLayout = DiscreteLayout(orientation='vertical')
        analogLayout = AnalogLayout(orientation='vertical')
        self.controlsLayout = ControlsLayout(orientation='vertical')


        # Add labels for discrete channels to left half of top level layout
        discreteLabels = DiscreteLabels()
        discreteLayout.add_widget(discreteLabels)

        # Add labels for analog channels to left half of top level layout
        analogLabels = AnalogLabels()
        analogLayout.add_widget(analogLabels)

        # Add discrete channel strips discrete sublayout
        self.discreteStrips = []
        for i in range(8):
            self.discreteStrips.append(ChannelStripDiscrete())
        for i in range(len(self.discreteStrips)):
            self.discreteStrips[i].channelID = i+1
            discreteLayout.add_widget(self.discreteStrips[i])

        # Add analog channel strips to analog sublayout
        self.analogStrips = []
        for i in range(4):
            self.analogStrips.append(ChannelStripAnalog())
        for i in range(len(self.analogStrips)):
            self.analogStrips[i].channelID = i+1
            analogLayout.add_widget(self.analogStrips[i])

        # Add sublayouts to main layout
        mainLayout.add_widget(discreteLayout)
        mainLayout.add_widget(analogLayout)
        mainLayout.add_widget(self.controlsLayout)

        return mainLayout


if __name__ == '__main__':
    app = ShockVibeBoxApp()
    app.run()
