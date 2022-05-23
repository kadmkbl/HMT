# import py
import netron

def main():
    # print('Hi from package_analyse.')
    # print("Read model")
    modelPath = "/home/wyz/ros2_catkin_ws/model_mobilenet.pth.tar"
    try:
        netron.start(modelPath)
    except :
        print("Read model fail!")

if __name__ == '__main__':
    main()
