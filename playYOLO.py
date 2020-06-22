import os, time
import argparse
import cv2
import numpy as np

#--------------------------------------------------------
modelType = "yolo"  
confThreshold = 0.4  #超過多少幾%的辨識率才會畫框框
nmsThreshold = 0.4   #若同個目標畫了太多框框，調整此值


classesFile = "yolo/robotpj.names" #存放class的名字
modelConfiguration = "yolo/robotpj.cfg"  #存放YOLO的設定檔
modelWeights = "yolo/robotpj_best.weights"    #訓練好的模型位置



displayScreen = True  #是否即時顯示預測結果
outputToFile = False   #是否輸出成檔案

#標籤和框框畫出來的參數
fontSize = 0.35
fontBold = 1
labelColor = (0,0,255)
boxbold = 1
boxColor = (255,255,255)
#--------------------------------------------------------

#圖片輸入模型會自動壓縮成這個解析度
if(modelType=="yolo"):
    inpWidth = 416      
    inpHeight = 416      
else:
    inpWidth = 320      
    inpHeight = 320     


#讀取我們放入的 class name 檔案
classes = None
with open(classesFile, 'rt') as f:
    classes = f.read().rstrip('\n').split('\n')

#載入YOLO的model
print("[INFO] loading YOLO from disk...")
net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)

#設定硬體運行的環境
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

#抓取我們在命令列輸入的檔名、相片模式或影片模式
parser = argparse.ArgumentParser(description="Test an image or a video?")
parser.add_argument("-i", dest='image', action='store', help='Image')
parser.add_argument("-v", dest='video', action='store',help='Video file')
#-----------------------------------------------------------------



#YOLO裡有非常多層類神經網路(DNN)，利用這個函式獲得最後一層輸出結果
def getOutputsNames(net):
    layersNames = net.getLayerNames()

    return [layersNames[i[0] - 1] for i in net.getUnconnectedOutLayers()]

#!!! 此函式從輸出層獲得每個目標在圖片裡的座標，然後對每個目標分類
def postprocess(frame, outs, orgFrame):
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
            if confidence > confThreshold:
                center_x = int(detection[0] * frameWidth)
                center_y = int(detection[1] * frameHeight)
                width = int(detection[2] * frameWidth)
                height = int(detection[3] * frameHeight)
                left = int(center_x - width / 2)
                top = int(center_y - height / 2)
                classIds.append(classId)
                confidences.append(float(confidence))
                boxes.append([left, top, width, height])  #每個目標都有一個box，內含座標
    
    indices = cv2.dnn.NMSBoxes(boxes, confidences, confThreshold, nmsThreshold) #這裡就用到了NMS，如果目標被重複框選，就會消成一個
    for i in indices:
        i = i[0]
        box = boxes[i]
        left = box[0]
        top = box[1]
        width = box[2]
        height = box[3]
        drawPred(classIds[i], confidences[i], left, top, left + width, top + height, orgFrame)

def Myfinction():
    print('設計函式\n')


#用openCV畫畫的函式，畫了一個框框和寫了LABEL上去
def drawPred(classId, conf, left, top, right, bottom, orgFrame):
    label = '%.2f' % conf
    labelName = '%s:%s' % (classes[classId], label)

    classtype=classes[classId]

    cv2.rectangle(frame, (left, top), (right, bottom), boxColor, boxbold)    #畫框框
    cv2.putText(frame, labelName, (left, top-10), cv2.FONT_HERSHEY_COMPLEX, fontSize, labelColor, fontBold)    #寫字

    #print(labelName)
            
 
    
#---選擇影像輸入模式，設定cap
args = parser.parse_args()
if (args.image):
    # Open the image file
    if not os.path.isfile(args.image):
        print("Input image file ", args.image, " doesn't exist")
        sys.exit(1)
    cap = cv2.VideoCapture(args.image)    #把image存入cap這個變數裡
    outputFile = args.image[:-4]+'_yolo.jpg'    #設定輸出的檔案名稱

elif (args.video):
    # Open the video file
    if not os.path.isfile(args.video):
        print("Input video file ", args.video, " doesn't exist")
        sys.exit(1)
    cap = cv2.VideoCapture(args.video)    #把video存入cap這個變數裡
    outputFile = args.video[:-4]+'_yolo.avi'    #設定輸出的檔案名稱
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    out = cv2.VideoWriter(outputFile, fourcc, 30.0, (round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))))

else:
    # Webcam input
    cap = cv2.VideoCapture(0)


	


#終於完成定義與參數設定，開始無限迴圈跑每一張影像
i = 0
while cv2.waitKey(1) < 0:
    hasFrame, frame = cap.read()   #從image or video or webcam 讀入一張

    i += 1 

    #如果沒有影像了，就 break loop
    if not hasFrame:
        print("Done processing !!!")
        print("Output file is stored as ", outputFile)
        cv2.waitKey(3000)
        break

    orgFrame = frame.copy()


    blob = cv2.dnn.blobFromImage(frame, 1/255, (inpWidth, inpHeight), [0,0,0], 1, crop=False)  #將三維圖片轉換成DNN在訓練的格式
    net.setInput(blob)   #設定DNN輸入
    outs = net.forward(getOutputsNames(net))   #blob會經過每一層的轉換、計算
    postprocess(frame, outs, orgFrame)  #最後獲得偵測目標與座標

    t, _ = net.getPerfProfile()
    
    

    #輸出成圖片
    if (args.image):

        if(outputToFile):
            cv2.imwrite(outputFile, frame.astype(np.uint8))

        if(displayScreen):
            cv2.imshow("Predicted", frame)
    #輸出成影片，以及判斷是否即時顯示視窗
    elif (args.video):
        print("Frame #{} processed.".format(i))

        if(outputToFile):
            out.write(frame)

        if(displayScreen):
            cv2.imshow("frame", frame)
            cv2.waitKey(1)
    else:
        if(displayScreen):
            cv2.imshow("frame", frame)
            cv2.waitKey(1)
    