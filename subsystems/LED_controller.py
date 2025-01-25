import wpilib
# import wpilib.drive
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

# import phoenix6

class LED_controller():

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
        
    # def GETOUT(self):
    #     # self.candle.animate(FireAnimation(0,0,0,0,0,False,0))
    #     # self.candle.animate(LarsonAnimation(0,0,0,0,0,0,phoenix5.led.LarsonAnimation.BounceMode(0),0,0))
    #     # self.candle.animate(StrobeAnimation(0,0,0,0,0,0,0))
    #     # self.candle.animate(ColorFlowAnimation(0,0,0,0,0,0,phoenix5.led.ColorFlowAnimation.Direction(0),0))
    #     # self.candle.animate(SingleFadeAnimation(0,0,0,0,0,0,0))
    #     # self.candle.animate(RgbFadeAnimation(0,0,0,0))
    #     # self.candle.animate(TwinkleAnimation(0,0,0,0,0,0,phoenix5.led.TwinkleAnimation.TwinklePercent(0),0))
    #     # self.candle.animate(TwinkleOffAnimation(0,0,0,0,0,0,phoenix5.led.TwinkleOffAnimation.TwinkleOffPercent(0),0))
    #     # self.candle.animate(RainbowAnimation(0,0,0,False,0))
    #     self.candle.setLEDs(0,0,0)
     
      #gets animations???
    def LED_AP(self):
      # LED Algae Pickup Color
        self.candle.setLEDs(0,150,50,0,0,153)
    
    def LED_CP(self):
      # LED Coral Pickup Color    
        self.candle.setLEDs(200,10,20,0,0,154)
   
    def LED_hang(self):
      # LED robot hang Color
        pass
    def LED_AA(self):
      # LED robot arm angle Color
        pass
    def LED_TF(self):
      # LED teutonic force color when chilling
     
      #self.candle.animate(LarsonAnimation(0,200,50,0,45,-1,phoenix5.led.LarsonAnimation.BounceMode(2),2,0))
      #self.candle.animate(StrobeAnimation(5,5,50,0,1,82,0))
      #self.candle.animate(ColorFlowAnimation(1,1,225,0,.25,308,phoenix5.led.ColorFlowAnimation.Direction(0),0))
      #self.candle.animate(FireAnimation(.80,.5,200,.35,.35,True,0))
      #self.candle.animate(SingleFadeAnimation(0,100,100,0,.5,308,0))
      #self.candle.animate(RgbFadeAnimation(.89,.5,-1,0))
      #self.candle.animate(TwinkleAnimation(100,0,100,0,.5,-1,phoenix5.led.TwinkleAnimation.TwinklePercent(100),0))
      #self.candle.animate(TwinkleOffAnimation(100,100,100,0,.5,-1,phoenix5.led.TwinkleOffAnimation.TwinkleOffPercent(100),0))
      #self.candle.animate(RainbowAnimation(.87,.5,-1,False,0))

      # A bunch of animations that we have !!! we can import more if we have to
      # we might also want to create custom animations
        
      #self.candle.animate(ColorFlowAnimation(0,200,40,0,.5,154,phoenix5.led.ColorFlowAnimation.Direction(1),0))
      pass
  
      
    def set_alliance_color(self, isRed:bool):
        if isRed == True: 
            self.candle.setLEDs(100,0,0,0,154,154)
        else:
            self.candle.setLEDs(0,0,100,0,154,154)
            
        