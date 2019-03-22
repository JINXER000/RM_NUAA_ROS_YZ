# -*- coding: utf-8 -*-
"""
Created on Thu Nov 15 22:45:01 2018

@author: CHEN YIZHOU
"""

import os
import numpy as np
from xml.parsers.expat import ParserCreate
import json
import xml.sax

#遍历文件夹，得到文件列表
def get_xmlLsit(rootDir,secDir):
    filePath=os.path.join(rootDir,secDir)
    xmlList=[]
    for root,dirs,files in os.walk(filePath):
        xmlList.extend([os.path.join(root,x) for x in files if (os.path.isfile(os.path.join(root,x))) and os.path.splitext(x)[1]=='.xml' and not "blue" in os.path.join(root,x)])
    return xmlList

def xml2line(xmlPath):
    armorDict=parseXml(xmlPath)
    tmpline=''
    line=''
    for idx in range(armorDict['idx']):
        tmpline=tmpline+armorDict['armor'][idx]['xmin']+'  '+armorDict['armor'][idx]['ymin']+'  '+armorDict['armor'][idx]['width']+'  '+armorDict['armor'][idx]['height']+'  '
    line=xmlPath.split('.')[0]+'.jpg  '+str(armorDict['idx'])+'  '+tmpline
    return line


def write_dat(xmlList,datPath):
    for xmlPath in xmlList:
        line=xml2line(xmlPath)+'\n'
        with open(datPath,'a') as f:
            f.write(line)

def xml2json(xmlList,datPath):
    wholeDict={}
    wholeDict["X"]=[]
    wholeDict["Y"]=[]
    wholeDict["path"]=[]
    for xmlPath in xmlList:
        armorDict=parseXml(xmlPath)
        for arm_num in range(1,6):
            for armX in armorDict["armor-%(num)d"%{"num":arm_num}]:
                wholeDict["X"].append(armX)
                wholeDict["Y"].append(arm_num)
                wholeDict["path"].append(xmlPath)
        for non in armorDict["non"]:
            wholeDict["X"].append(non)
            wholeDict["Y"].append(0)
            wholeDict["path"].append(xmlPath)
        
    with open(datPath,'a') as f:
#        jsonInfo=json.dumps(wholeDict)
#        json.dump(jsonInfo,f)
        json.dump(wholeDict,f)
        
 #解析xml的handler
class labelHandler( xml.sax.ContentHandler ):
    def __init__(self):
        self.CurrentData = ""
        self.name = ""
        self.xmin = ""
        self.ymin = ""
        self.width = ""
        self.height = ""
        
        self.result={}
        self.result['armor-1']=[]
        self.result['idx-1']=0
        self.result['armor-2']=[]
        self.result['idx-2']=0
        self.result['armor-3']=[]
        self.result['idx-3']=0
        self.result['armor-4']=[]
        self.result['idx-4']=0
        self.result['armor-5']=[]
        self.result['idx-5']=0
        self.result['non']=[]
        self.result['idx-non']=0        
        # 元素开始事件处理
    def startElement(self, tag, attributes):
        self.CurrentData = tag
 
   # 元素结束事件处理
    def endElement(self, tag):
        if self.CurrentData == "name":
            print("name",self.name)
           
        elif self.CurrentData == "xmin":
            print ("xmin:", self.xmin)
        elif self.CurrentData == "ymin":
            print ("ymin:", self.ymin)
        elif self.CurrentData == "xmax":
            print ("width:", self.width)
        elif self.CurrentData == 'ymax':
            print ("height:", self.height)
        elif tag=='object':
            if self.name=='armor-1':
                self.result['idx-1']+=1
                armor={
                  'xmin':str(self.xmin),
                  'ymin':str(self.ymin),
                  'height':str(self.height),
                  'width':str(self.width),
                  }
                self.result['armor-1'].append(armor)
            elif self.name=='armor-2':
                self.result['idx-2']+=1
                armor={
                  'xmin':str(self.xmin),
                  'ymin':str(self.ymin),
                  'height':str(self.height),
                  'width':str(self.width),
                  }
                self.result['armor-2'].append(armor)
            elif self.name=='armor-3':
                self.result['idx-3']+=1
                armor={
                  'xmin':str(self.xmin),
                  'ymin':str(self.ymin),
                  'height':str(self.height),
                  'width':str(self.width),
                  }
                self.result['armor-3'].append(armor)
            elif self.name=='armor-4':
                self.result['idx-4']+=1
                armor={
                  'xmin':str(self.xmin),
                  'ymin':str(self.ymin),
                  'height':str(self.height),
                  'width':str(self.width),
                  }
                self.result['armor-4'].append(armor)
            elif self.name=='armor-5':
                self.result['idx-5']+=1
                armor={
                  'xmin':str(self.xmin),
                  'ymin':str(self.ymin),
                  'height':str(self.height),
                  'width':str(self.width),
                  }
                self.result['armor-5'].append(armor)
            elif self.name=='non':
                self.result['idx-non']+=1
                armor={
                  'xmin':str(self.xmin),
                  'ymin':str(self.ymin),
                  'height':str(self.height),
                  'width':str(self.width),
                  }
                self.result['non'].append(armor)

        elif tag=='annotation':
            print('whole', self.result)
        self.CurrentData = ""
 
   # 内容事件处理
    def characters(self, content):
        if self.CurrentData == "name":
            self.name = content
        elif self.CurrentData == "xmin":
            self.xmin = int(content)
        elif self.CurrentData == "ymin":
            self.ymin = int(content)
        elif self.CurrentData == "xmax":
            self.width = int(content)-self.xmin
        elif self.CurrentData == "ymax":
            self.height = int(content)-self.ymin


#解析xml
def parseXml(xml_str):
    parser = xml.sax.make_parser()
       # turn off namepsaces
    parser.setFeature(xml.sax.handler.feature_namespaces, 0)
       # 重写 ContextHandler
    Handler = labelHandler()
    parser.setContentHandler( Handler )
    parser.parse(xml_str)
    return Handler.result

#生成负样本图片列表并写入txt
def gen_negtxt(rootDir,secDir,dataPath):
    filePath=os.path.join(rootDir,secDir)
    negList=[]
    for root,dirs,files in os.walk(filePath):
        negList.extend([os.path.join(secDir,x) for x in files if (os.path.isfile(os.path.join(root,x))) and os.path.splitext(x)[1]=='.jpg'])
    for negPath in negList:
        line=negPath+'\n'
        with open(dataPath,'a') as f:
            f.write(line)            
    
        
    
if __name__=='__main__':
#   xmlPath='img5.xml'
#   line=xml2line(xmlPath)
    #路径是 rootDir+secDir,会遍历每一个xml文件
#     rootDir='F:\CV_WS\SAMPLEs\RM19-ARMOR'    # root directry of xml files
#     secDir='2345-indoor'   # folder contains xmls
#     datPath='pos2345-indoor.txt'  #generated txt, 和py在一个文件夹
    rootDir=r"/home/nvidia/yzchen_ws/data0107"
    secDir=""
    out_json=os.path.join(rootDir,"Red_numberDetect.json")
    xmlList=get_xmlLsit(rootDir,secDir)
#     write_dat(xmlList,datPath)
    xml2json(xmlList,out_json)
#     gen_negtxt(rootDir,'neghaar','neghaar.txt')
