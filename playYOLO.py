import os
import time
import argparse
import cv2
import numpy as np
from realsense_basic import run


class YOLO():
    def __init__(self, modelname, confidence):
        self.classesFile        = "yolo/" + modelname + ".names"    # 存放class的名字
        self.modelWeights       = "yolo/" + modelname + ".weights"  # 訓練好的模型位置
        self.modelConfiguration = "yolo/" + modelname + ".cfg"      # 存放YOLO的設定檔
        self.confThreshold = confidence
        # 讀取 class name 檔案
        self.classes = None
        with open(self.classesFile, 'rt') as f:
            self.classes = f.read().rstrip('\n').split('\n')

        # 載入YOLO的model
        print("[INFO] loading YOLO from disk...")
        self.net = cv2.dnn.readNetFromDarknet(self.modelConfiguration, self.modelWeights)

        # 設定硬體運行的環境
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    # YOLO裡有非常多層類神經網路(DNN)，利用這個函式獲得最後一層輸出結果
    def getOutputsNames(self):
        layersNames = self.net.getLayerNames()
        return [layersNames[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

    # !!! 此函式從輸出層獲得每個目標在圖片裡的座標，然後對每個目標分類
    def postprocess(self, frame, outs, orgFrame):
        # 用openCV畫畫的函式，畫了一個框框和寫了LABEL上去
        def drawPred(classId, conf, left, top, right, bottom, orgFrame):
            # 標籤和框框的參數
            fontSize = 0.35
            fontBold = 1
            labelColor = (0, 0, 255)
            boxbold = 1
            boxColor = (255, 255, 255)
            # --------------------------------------------------------

            label = '%.2f' % conf
            labelName = '%s:%s' % (self.classes[classId], label)

            classtype = self.classes[classId]

            cv2.rectangle(orgFrame, (left, top), (right, bottom), boxColor, boxbold)  # 畫框框
            cv2.putText(orgFrame, labelName, (left, top-10), cv2.FONT_HERSHEY_COMPLEX, fontSize, labelColor, fontBold)  # 寫字
            # print(labelName)

        # --------------------------------------------------------

        nmsThreshold = 0.8   # 若同個目標畫了太多框框，調整此值
        frameHeight = frame.shape[0]
        frameWidth = frame.shape[1]

        classIds = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                classId = np.argmax(scores)
                confidence = scores[classId]
                if confidence > self.confThreshold:  # 超過多少幾%的辨識率才會畫框框
                    center_x = int(detection[0] * frameWidth)
                    center_y = int(detection[1] * frameHeight)
                    width = int(detection[2] * frameWidth)
                    height = int(detection[3] * frameHeight)
                    left = int(center_x - width / 2)
                    top = int(center_y - height / 2)
                    classIds.append(classId)
                    confidences.append(float(confidence))
                    boxes.append([left, top, width, height])  # 每個目標都有一個box，內含座標

        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confThreshold, nmsThreshold)  # 這裡就用到了NMS，如果目標被重複框選，就會消成一個
        centers = []

        for i in indices:
            i = i[0]
            box = boxes[i]
            left = box[0]
            top = box[1]
            width = box[2]
            height = box[3]
            drawPred(classIds[i], confidences[i], left, top, left + width, top + height, orgFrame)
            centers.append([left + width/2, top + height/2])
        return centers

    def detect(self, color_image, depth_image):
        blob = cv2.dnn.blobFromImage(color_image, 1 / 255, (256, 256), [0, 0, 0], 1, crop=False)  # 將三維圖片轉換成DNN在訓練的格式
        self.net.setInput(blob)  # 設定DNN輸入
        outs = self.net.forward(self.getOutputsNames())  # blob會經過每一層的轉換、計算
        centers = self.postprocess(color_image, outs, color_image)  # 最後獲得偵測目標與座標
        return centers


if __name__ == "__main__":
    net = YOLO("platform_tiny")
    # net = YOLO("yolov3_hole")

    def runNet(net):
        def test(color_image, depth_image):
            if color_image is None:
                return False, None
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.3), cv2.COLORMAP_JET)
            centers = net.detect(color_image)
            print(centers)
            return False, np.hstack((depth_colormap, color_image))
        return test

    run(runNet(net))
