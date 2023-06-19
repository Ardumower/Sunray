# OpenCV heatmap image 

import numpy as np
import cv2
import os
from PIL import Image
import subprocess


class HeatImageCV():
    def __init__(self, name, minVal, maxVal, text, colormap = None, medianInterpolate = False):
        self.name = name
        self.imgW = 1
        self.imgH = 1
        self.img = None
        self.mask = None
        self.legend = None
        self.frames = []
        self.videoWriter = None
        self.minX = 0
        self.maxX = 0
        self.minY = 0
        self.maxY = 0
        self.resize = 5
        self.minVal = minVal
        self.maxVal = maxVal
        self.text = text
        self.colormap = colormap
        self.doFillGaps = True
        self.makeGif = False
        self.makeVideo = False
        self.outputPath = 'output'
        self.medianInterpolate = medianInterpolate        
        if colormap is None:
            self.colormap =  np.zeros((256,1,3), dtype=np.uint8)         
            for idx in range(0, 256): 
                self.colormap[idx, 0, 0] = ((1+idx) * 133) % 255
                self.colormap[idx, 0, 1] = ((1+idx) * 266) % 255
                self.colormap[idx, 0, 2] = ((1+idx) * 333) % 255
        if not os.path.exists(self.outputPath):         
            os.mkdir(self.outputPath)        
            

    def resizeMinMax(self, minX, maxX, minY, maxY):
        #print('resizeMinMax', minX, maxX, minY, maxY)
        self.minX = minX
        self.maxX = maxX
        self.minY = minY
        self.maxY = maxY        
        self.imgW = int(round(abs(maxX-minX)))
        self.imgH = int(round(abs(maxY-minY)))
        self.mask = np.zeros((self.imgH,self.imgW), dtype=np.uint8) 
        self.img = np.zeros((self.imgH,self.imgW), dtype=np.uint8) 
        

    # for a given coordinate, pad image if coordinate is outside image dim 
    def padImage(self, x, y):
        #global imgW, imgH, imgSig, imgPing, minX, maxX, minY, maxY
        if self.img is None:
            self.img = np.zeros((self.imgH,self.imgW), dtype=np.uint8) 
            self.mask = np.zeros((self.imgH,self.imgW), dtype=np.uint8)  
            self.minX = x
            self.maxX = x
            self.minY = y
            self.maxY = y
        xleft = abs(min(self.minX, x) - self.minX)
        xright = abs(max(self.maxX, x) - self.maxX)
        ytop = abs(min(self.minY, y) - self.minY)
        ybottom = abs(max(self.maxY, y) - self.maxY)
        nh = ytop + ybottom +self.imgH    
        nw = xleft + xright +self.imgW
        
        # Create a white image with the target size    
        padded_image = np.zeros((nh,nw), dtype=np.uint8) 
        # Add the image to the padded image, with padding on the sides
        padded_image[ytop:ytop+self.imgH, xleft:xleft+self.imgW] = self.img
        self.img = padded_image                

        # Create a white image with the target size    
        padded_image = np.zeros((nh,nw), dtype=np.uint8) 
        # Add the image to the padded image, with padding on the sides
        padded_image[ytop:ytop+self.imgH, xleft:xleft+self.imgW] = self.mask
        self.mask = padded_image                                
        
        self.minX = min(self.minX, x)
        self.maxX = max(self.maxX, x)
        self.minY = min(self.minY, y)
        self.maxY = max(self.maxY, y)
        self.imgW = nw
        self.imgH = nh
        # print('x', x, 'y', y, 'minX', self.minX, 'maxX', self.maxX, 'minY', self.minY, 'maxY', self.maxY, 'w', self.imgW, 'h', self.imgH)    

    def drawPoint(self, px, py, val):
        overlap = 0
        for y in range(py-overlap, py+overlap+1):
            for x in range(px-overlap, px+overlap+1):        
                if y < 0 or y >= self.imgH or x < 0 or x >= self.imgW: continue 
                #if self.mask[y, x] != 0: continue
                self.mask[y, x] = 1
                self.img[y, x] = val

    def fillGaps(self):
        nimg = self.img.copy()
        nmask = self.mask.copy()        
        overlap = 1
        filled = 0
        for py in range(0, self.imgH):
            for px in range(0, self.imgW):            
                if nmask[py, px] != 0: continue
                valSum = 0
                valCount = 0
                valList = []                
                for y in range(py-overlap, py+overlap+1):
                    for x in range(px-overlap, px+overlap+1):        
                        if y < 0 or y >= self.imgH or x < 0 or x >= self.imgW: continue 
                        if self.mask[y, x] == 0: continue                                                
                        valSum += self.img[y, x]
                        valCount += 1
                        valList.append(self.img[y, x])
                if valCount == 0: continue
                valAvg = valSum / valCount            
                #print(valCount, valAvg)                
                if self.medianInterpolate:
                    valList.sort()
                    nimg[py, px] = valList[ len(valList) // 2 ] 
                else:
                    nimg[py, px] = int(round(valAvg))    
                nmask[py, px] = 1
                filled += 1
        # print('filled gaps', filled)
        #self.img = nimg     
        #self.mask = nmask                    
        return (nimg, nmask)        

    # get color for value
    def valToCol(self, val):
        minVal = float(self.minVal)
        maxVal = float(self.maxVal)        
        n = max(minVal, min(maxVal, val)) # crop value to min/max range
        n = abs(n - minVal) / abs(minVal-maxVal) * 255.0    
        col = int(round( n ))
        return col         

    # draw signal
    def drawSignal(self, x, y, val):
        cx = int(round(x) - self.minX)
        cy = int(round(y) - self.minY)
        col = self.valToCol(val)
        # print(cx, cy, col)
        self.drawPoint(cx, cy, col)
        if self.makeGif or self.makeVideo:
            self.addAnimationFrame(self.img, self.mask)


    def createLegend(self, height):
        stepH = 20
        valSteps = int(height / stepH)        
        w = 210
        stepVal = float(self.maxVal - self.minVal) / float(valSteps)
        #print('minVal', self.minVal, 'maxVal', self.maxVal, 'stepVal', stepVal, 'valSteps', valSteps, )        
        img = np.zeros((height,w), dtype=np.uint8) 
        y = height
        x = 0
        val = self.minVal
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        # fontScale
        fontScale = 0.5   
        # Line thickness of 2 px
        thickness = 1
           
        for i in range(0, valSteps+1):                
            if self.medianInterpolate:
                col = self.valToCol(int(val))
            else:
                col = self.valToCol(val)
            #print(i, val, col)            
            cv2.rectangle(img, (x, y), (x + w-1, y - stepH), (col), -1)
            #img = cv2.putText(img, str(val) + ' ' + self.text, (x, y+15), font, fontScale, (col+50) % 255, thickness, cv2.LINE_AA)    
            y -= stepH
            val += stepVal
        imgCol = cv2.applyColorMap(img, self.colormap)
        
        y = height
        val = self.minVal
        for i in range(0, valSteps+1):                
            s = ''
            if self.medianInterpolate:
                n = int(val)
            else:
                n = round(val, 1)
            if isinstance(self.text, list):                
                if n < len(self.text): s = self.text[n]                
            else:
                s = self.text
            imgCol = cv2.putText(imgCol, str(n) + ' ' + s, (x, y-stepH+20), font, fontScale, 0, thickness, cv2.LINE_AA)    
            y -= stepH
            val += stepVal
        #cv2.imwrite('legend.png', imgCol)        
        return imgCol

    def saveImage(self):
        if not self.videoWriter is None: self.videoWriter.release()            
        filename = os.path.join(self.outputPath, 'wifi_heatmap_' + self.name + '.jpg')

        if self.doFillGaps:
            nimg, nmask =  self.fillGaps()
            self.img = nimg     
            self.mask = nmask                            
            
        imgCol = cv2.applyColorMap(self.img, self.colormap)

        # apply mask
        idx = (self.mask==0)
        imgCol[idx] = 0

        # flip vertically        
        imgCol = cv2.flip(imgCol, 0)
                
        dim = (self.imgW * self.resize, self.imgH * self.resize)        
        resized = cv2.resize(imgCol, dim, interpolation = cv2.INTER_AREA)

        if self.legend is None:
            self.legend = self.createLegend(resized.shape[0])
        #print(resized.shape, legend.shape)
        concat = cv2.hconcat([resized, self.legend])
        
        h, w, _ = concat.shape    
        print('saving image: %s (%dx%d)' % (filename, w, h) )                                                        
        cv2.imwrite(filename, concat)

        if self.makeGif:
            if len(self.frames) > 0:
                frame_one = self.frames[0]
                h, w = frame_one.size
                filename = os.path.join(self.outputPath, 'wifi_heatmap_' + self.name + '.gif')                
                print('saving gif: %s (%dx%d)' % (filename, w, h) )                                                        
                frame_one.save(filename, format="GIF", append_images=self.frames, save_all=True, duration=0.1, loop=0)

                #if self.makeVideo:
                #    outfilename = os.path.join(self.outputPath, 'wifi_heatmap_' + self.name + '.mp4')                                                
                #    cmd = 'ffmpeg -y -i ' + filename + ' -r 16 -filter:v "setpts=0.25*PTS"  ' + outfilename
                #    print('CMD:' + cmd)
                #    try:
                #       output = subprocess.check_output(cmd, shell=True)
                #       output = output.decode()
                #    except Exception as e:
                #       print("Error creating video: " + str(e))


    def addAnimationFrame(self, nimg, nmask):
        #nimg, nmask =  self.fillGaps()            
        imgCol = cv2.applyColorMap(nimg, self.colormap)

        # apply mask
        idx = (nmask==0)
        imgCol[idx] = 0

        # flip vertically        
        imgCol = cv2.flip(imgCol, 0)
                
        dim = (self.imgW * self.resize, self.imgH * self.resize)        
        resized = cv2.resize(imgCol, dim, interpolation = cv2.INTER_AREA)

        if self.legend is None:
            self.legend = self.createLegend(resized.shape[0])
        #print(resized.shape, legend.shape)
        concat = cv2.hconcat([resized, self.legend])
        
        if self.makeGif:
            pilImg = Image.fromarray(np.uint8(concat)).convert('RGB')
            self.frames.append(pilImg)

        if self.makeVideo:
            if self.videoWriter is None:                                
                outfilename = os.path.join(self.outputPath, 'wifi_heatmap_' + self.name + '.mkv')    
                h, w, _ = concat.shape    
                print('saving video: %s (%dx%d)' % (outfilename, w, h) )                    
                # https://stackoverflow.com/questions/30103077/what-is-the-codec-for-mp4-videos-in-python-opencv                                    
                self.videoWriter = cv2.VideoWriter(outfilename,cv2.VideoWriter_fourcc(*'avc1'), 100, (w, h))
            self.videoWriter.write(concat)

        #path = os.path.join(self.outputPath, 'gif')        
        #if not os.path.exists(path):         
        #    os.mkdir(path)        
        #path = os.path.join(path, self.name)
        #if not os.path.exists(path):         
        #    os.mkdir(path)        
        #filename = os.path.join(path, "%05d.jpg" % (self.animFrameCount))
        #cv2.imwrite(filename, concat)      
        
        

if __name__ == "__main__":
    hi = HeatImageCV()
    hi.padImage(-10, -10)    
    hi.padImage(10, 10)
    hi.drawSignal(0,0,  100,  0, 100)
    hi.drawSignal(10,0, 0,    0, 100)
    hi.saveImage('test')

