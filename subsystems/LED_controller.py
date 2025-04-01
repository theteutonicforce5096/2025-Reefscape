import wpilib
import phoenix5.led
from phoenix5.led import Animation
from phoenix5.led import LarsonAnimation
from phoenix5.led import FireAnimation
from phoenix5.led import ColorFlowAnimation
from phoenix5.led import TwinkleAnimation
from phoenix5.led import StrobeAnimation
from phoenix5.led import SingleFadeAnimation
from phoenix5.led import RainbowAnimation
from phoenix5.led import RgbFadeAnimation
from phoenix5.led import TwinkleOffAnimation


class LED_controller:

    # What controls the color of the LED
    def __init__(self):
        self.candle = phoenix5.led.CANdle(1)
        self.candle.configFactoryDefault()
        config = phoenix5.led.CANdleConfiguration()
        #  CANdleConfiguration config;
        config.stripType = phoenix5.led.LEDStripType.RGB
        config.brightnessScalar = 0.5
        # candle.ConfigAllSettings(config)
        self.candle.configAllSettings(config)

        self.candle.animate(
            ColorFlowAnimation(
                0, 0, 0, 0, 0, 0, phoenix5.led.ColorFlowAnimation.Direction(0), 0
            ),
            0,
        )
        self.candle.animate(
            ColorFlowAnimation(
                0, 0, 0, 0, 0, 0, phoenix5.led.ColorFlowAnimation.Direction(0), 0
            ),
            1,
        )

    #   ^^^ The programmers of the py library made a bug to phoenix5 SO
    #   ^^^ to clear the animation you have to redeploy THIS
    def LED_CP(self, have_coral: bool):
        # LED Coral Pickup Color
        if have_coral:
            self.candle.setLEDs(200, 10, 20, 0, 154, 154)
        else:
            self.shine_alliance_color()

    def LED_hang(self, have_hang: bool):
        # LED robot hang Color
        if have_hang:
            self.candle.animate(
                LarsonAnimation(
                    0,
                    150,
                    50,
                    0,
                    0.75,
                    77,
                    phoenix5.led.LarsonAnimation.BounceMode(0),
                    7,
                    154,
                ),
                0,
            )
            self.candle.animate(
                LarsonAnimation(
                    0,
                    150,
                    50,
                    0,
                    0.75,
                    77,
                    phoenix5.led.LarsonAnimation.BounceMode(0),
                    7,
                    231,
                ),
                1,
            )
        else:
            self.shine_alliance_color()

    def LED_AA1(self, have_angle1: bool):
        # LED robot arm angle Color
        if have_angle1:
            self.candle.setLEDs(225, 225, 225, 0, 154, 154)
        else:
            self.alliance_color()

    def LED_AA2(self, have_angle2: bool):
        # LED robot arm angle Color
        if have_angle2:
            self.candle.setLEDs(10, 225, 0, 0, 154, 154)
        else:
            self.alliance_color()

    def LED_AA3(self, have_angle3: bool):
        # LED robot arm angle Color
        if have_angle3:
            self.candle.setLEDs(225, 0, 225, 0, 154, 154)
        else:
            self.alliance_color()

    def set_alliance_color(self, isRed: bool):
        if isRed:
            self.alliance_color = "red"
        else:
            self.alliance_color = "blue"
        self.shine_alliance_color()

    def shine_alliance_color(self):
        if self.alliance_color == "red":
            self.candle.setLEDs(100, 0, 0, 0, 0, 308)
        else:
            self.candle.setLEDs(0, 0, 100, 0, 0, 308)

    # +!!!  ANIMATIONS  !!!+
    # A bunch of animations that we have !!! we can import more if we have to
    # We might also want to create custom animations

    # self.candle.animate(LarsonAnimation(0,200,50,0,45,-1,phoenix5.led.LarsonAnimation.BounceMode(2),2,0))
    # self.candle.animate(StrobeAnimation(5,5,50,0,1,82,0))
    # self.candle.animate(ColorFlowAnimation(1,1,225,0,.25,308,phoenix5.led.ColorFlowAnimation.Direction(0),0))
    # self.candle.animate(FireAnimation(.80,.5,200,.35,.35,True,0))
    # self.candle.animate(SingleFadeAnimation(0,100,100,0,.5,308,0))
    # self.candle.animate(RgbFadeAnimation(.89,.5,-1,0))
    # self.candle.animate(TwinkleAnimation(100,0,100,0,.5,-1,phoenix5.led.TwinkleAnimation.TwinklePercent(100),0))
    # self.candle.animate(TwinkleOffAnimation(100,100,100,0,.5,-1,phoenix5.led.TwinkleOffAnimation.TwinkleOffPercent(100),0))
    # self.candle.animate(RainbowAnimation(.87,.5,-1,False,0))
    # self.candle.animate(ColorFlowAnimation(0,200,40,0,.5,154,phoenix5.led.ColorFlowAnimation.Direction(1),0))
