from board import SCL, SDA
import busio
import adafruit_pca9685

def main():
    i2c = busio.I2C(1, 3, 2) # TODO: switch away from busio??? maybe use SMBus instead??
    pca = adafruit_pca9685.PCA9685(i2c)
    print('Hi from joint_ctl.')


if __name__ == '__main__':
    main()
