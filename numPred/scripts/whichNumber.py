# -*- coding: utf-8 -*-
"""
Created on Sat Feb  2 22:17:09 2019

@author: Jinxe
"""

import cv2
import json
import os
import numpy as np
import keras
from keras.preprocessing.image import ImageDataGenerator, array_to_img, img_to_array, load_img
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D
from keras.layers import Activation, Dropout, Flatten, Dense
import matplotlib.pyplot as plt
from keras.utils.vis_utils import plot_model
from xml2txt import parseXml

input_shape=(40,40,1)
resize_shape=(40,40)
data_augmentation=False
datagen = ImageDataGenerator(
        rotation_range=20,
        width_shift_range=0.2,
        height_shift_range=0.2,
        shear_range=0.2,
        zoom_range=0.2,
        horizontal_flip=False,
        fill_mode='nearest')

def test1XML(xmlPath):
    armorDict=parseXml(xmlPath)
    testDict={}
    testDict["X"]=[]
    testDict["Y"]=[]
    testX=[]
    for arm_num in range(1,6):
        for armX in armorDict["armor-%(num)d"%{"num":arm_num}]:
            testDict["X"].append(armX)
            testDict["Y"].append(arm_num)
            testDict["path"].append(xmlPath)
    totalNum=len(testDict["X"])
    for idx in range(totalNum):
        if int(testDict["X"][idx]["width"])<30 or int(testDict["X"][idx]["height"])<30:
            continue
                
        imgName=testDict["path"][idx].split('.')[0]+'.jpg'
        try:
            img=cv2.imread(imgName)
            left=int(testDict["X"][idx]["xmin"])
            right=left+int(testDict["X"][idx]["width"])
            up=int(testDict["X"][idx]["ymin"])
            down=up+int(testDict["X"][idx]["height"])
            imgPatch= img[up:down,left:right]
            imgPatch=cv2.resize(imgPatch,resize_shape)
        except:
            print("fuck"+imgName)
        testX.append(np.array(imgPatch))
    testX=np.array(testX)
    return testX,testDict["Y"]
    
def cropXandAug(jsonPath,saveDir):
    dataDict={}
    X=[]
    Y=[]
    
    with open(jsonPath) as f:
        dataDict=json.load(f)
        totalNum=len(dataDict["X"])
        for idx in range(totalNum):
            if int(dataDict["X"][idx]["width"])<30 or int(dataDict["X"][idx]["height"])<30:
                continue
                    
            imgName=dataDict["path"][idx].split('.')[0]+'.jpg'
            try:
                img=cv2.imread(imgName)
                left=int(dataDict["X"][idx]["xmin"])
                right=left+int(dataDict["X"][idx]["width"])
                up=int(dataDict["X"][idx]["ymin"])
                down=up+int(dataDict["X"][idx]["height"])
                imgPatch= img[up:down,left:right]
                imgPatch=cv2.resize(imgPatch,resize_shape)
            except:
                print("fuck"+imgName)
            label=dataDict["Y"][idx]
            prefix="armor_"+str(label)+"-"
            dataAugment(imgPatch,saveDir,prefix)
            imgPatch=np.array(imgPatch)
            X.append(imgPatch)
            Y.append(label)
    X_array=np.array(X)
    Y_array=np.array(Y)
    return X_array,Y_array

def get_data_from_aug(aug_dir):
    X=[]
    Y=[]
    pic_list=[]
    for root,dirs,files in os.walk(aug_dir):
        for x in files:
            if (os.path.isfile(os.path.join(root,x))) and os.path.splitext(x)[1]=='.jpeg':
                pic_list.append(os.path.join(root,x))
    
    for pic_path in pic_list:

        label=pic_path.split('-')[0].split('_')[-1]
        if label=='0':
            continue
        data=cv2.imread(pic_path,0)
        data=cv2.resize(data,resize_shape)
        # img_eq=cv2.equalizeHist(data)
        # _,img_th=cv2.threshold(img_eq,128,255,cv2.THRESH_BINARY)
        # cv2.imshow('EQ',img_eq)
        # cv2.waitKey(0)
        X.append(data)
        Y.append(label)
    X_array=np.array(X)
    Y_array=np.array(Y)
    return X_array,Y_array
        


            
def dataAugment(imgPatch,save_dir,prefix):
    x=img_to_array(imgPatch)
    x=x.reshape((1,)+x.shape)
    i = 0
    for batch in datagen.flow(x, batch_size=1,
                          save_to_dir=save_dir, save_prefix=prefix, save_format='jpeg'):
        i += 1
        if i > 20:
            break  # otherwise the generator would loop indefinitely




def modelStructor(inputShape,num_classes):
    model = Sequential()
    model.add(Conv2D(32, kernel_size=(3, 3),
                    activation='relu',
                    input_shape=input_shape))
    model.add(Conv2D(64, (3, 3), activation='relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Dropout(0.25))
    model.add(Flatten())
    model.add(Dense(128, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(num_classes, activation='softmax'))
    # model = Sequential()
    # model.add(Conv2D(32, (3, 3), padding='same',
    #              input_shape=inputShape))
    # model.add(Activation('relu'))
    # model.add(Conv2D(32, (3, 3)))
    # model.add(Activation('relu'))
    # model.add(MaxPooling2D(pool_size=(2, 2)))
    # model.add(Dropout(0.25))
    # model.add(Conv2D(64, (3, 3), padding='same'))
    # model.add(Activation('relu'))
    # model.add(Conv2D(64, (3, 3)))
    # model.add(Activation('relu'))
    # model.add(MaxPooling2D(pool_size=(2, 2)))
    # model.add(Dropout(0.25))
    
    # model.add(Flatten())
    # model.add(Dense(128))
    # model.add(Activation('relu'))
    # model.add(Dropout(0.5))
    # model.add(Dense(num_classes))
    # model.add(Activation('softmax'))
    
    # initiate RMSprop optimizer
    opt = keras.optimizers.rmsprop(lr=0.005, decay=1e-6)
    
    # Let's train the model using RMSprop
    model.compile(loss='categorical_crossentropy',
                  optimizer=opt,
                  metrics=['accuracy'])
    return model
    
def shufandSplit(X_data,Y_data,num_classes):
    # np.expand_dims(X_data,axis=3)
    np.random.seed(1024)
    data_index=[i for i in range( X_data.shape[0])]
    np.random.shuffle(data_index)
    X_data_sf=X_data[data_index]
    Y_data_sf=Y_data[data_index]
    splitpoint = int(round(len(data_index) * 0.9))
    (X_train, X_val) =(X_data_sf[0:splitpoint],X_data_sf[splitpoint:])
    (Y_train, Y_val)=(Y_data_sf[0:splitpoint],Y_data_sf[splitpoint:])
    Y_oh_train=keras.utils.to_categorical(Y_train, num_classes)
    Y_oh_val=keras.utils.to_categorical(Y_val, num_classes)
    return X_train,X_val,Y_oh_train,Y_oh_val




if __name__=='__main__':
    model_path="/home/nvidia/yzchen_ws/ros_ws/src/numPred/models/numberClassify_Redmodel_min.h5"
    testImgPath="/home/nvidia/yzchen_ws/data0107/aug_save/armor_3-_0_9872.jpeg"
    aug_path="/home/nvidia/yzchen_ws/data0107/aug_save"
    batch_size=32
    epochs=50
    num_classes=6
    plot_histroy=1
    istrained=0
    

    if not istrained:

        X_data,Y_data=get_data_from_aug(aug_path)
        X_data=X_data.reshape(X_data.shape[0],40,40,1)
        X_train,X_val,Y_oh_train,Y_oh_val=shufandSplit(X_data,Y_data,num_classes)
        X_train=X_train/255
        X_val=X_val/255



        model=modelStructor(input_shape,num_classes)
        model.summary()
        if not data_augmentation:
            print('Not using data augmentation.')
            history=model.fit(X_train,Y_oh_train,
                      batch_size=batch_size,epochs=epochs,
                      validation_data=(X_val,Y_oh_val))
        else:
            print('Using real-time data augmentation.')
            datagen.fit(X_train)
            history=model.fit_generator(datagen.flow(X_train,Y_oh_train,batch_size=batch_size),
                                epochs=epochs,validation_data=(X_val,Y_oh_val), workers=4)
        
        
        model.save(model_path)
    
    	##plot acc and loss 
        if plot_histroy:
            fig = plt.figure()
            plt.title('model accuracy')
            plt.ylabel('accuracy')
            plt.xlabel('epoch')
            plt.plot(history.history['acc'])
            plt.plot(history.history['loss'])
            plt.title('model loss and accuracy')
            plt.ylabel('value')
            plt.xlabel('epoch')
            plt.legend(["acc","loss"])
            fig.savefig('performance.png')
    else:
        
        model=keras.models.load_model(model_path)
    # model=keras.models.load_model(model_path)
#    plot_model(model, to_file='Cnn_model.png', show_shapes=True)
    # testImg_l,testLabel=test1XML(testXmlPath) 
    testImg=cv2.imread(testImgPath)
    testImg=cv2.resize(testImg,resize_shape)
    testImg_a=np.array(testImg)
    testImg_l=[]
    testImg_l.append(testImg_a)
    testImg_al=np.array(testImg_l)
    pred=model.predict(testImg_al)
    num = np.argmax(pred)
    print(num)