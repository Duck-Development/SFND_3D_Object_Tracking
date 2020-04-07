import sys
import os

def main():
  frames ={}
  analyseCVS = ''
  
  filepath = sys.argv[1]
  print ("Open File")
  print (filepath)
  detector = ""
  descripor = "" 
  with open(filepath) as fp:
    for line in fp:
      if line.startswith('###Activ Detector: '):
        lineA = line.split('\n')
        lineA = lineA[0].split(' ')
        detector = lineA[2]
      
      if line.startswith('###Activ Descriptor: '):
        lineA = line.split('\n')
        lineA = lineA[0].split(' ')
        descriptor = lineA[2]
      
      if line.startswith('Pocess DataSet: '):  
        lineA = line.split('\n')
        lineA = lineA[0].split(' ')
        frame = lineA[2]
        if not (frame in frames):
          frames[frame] = {}
      
      if line.startswith('###LIDAR_TTC'):
        lineA = line.split('\n')
        lineA = lineA[0].split(':')
        lidar_ttc = lineA[1]
 
        if not (detector in frames[frame]):
          frames[frame][detector] = {}

        if not (descriptor in frames[frame][detector]):
          frames[frame][detector][descriptor] = {}

        frames[frame][detector][descriptor]['LIDAR_TTC'] = lidar_ttc

      if line.startswith('###CAM_TTC'):
        lineA = line.split('\n')
        lineA = lineA[0].split(':')
        cam_ttc = lineA[1]
        frames[frame][detector][descriptor]['CAM_TTC'] = cam_ttc

    #print( frames)    
    #exit(0)

##full data parse ready
    det = [ 'HARRIS' , 'SHITOMASI' , 'ORB' , 'SIFT' , 'FAST', 'BRISK' , 'AKAZE'  ]
    desc = [ 'BRISK' ,  'BRIEF' ,  'ORB' , 'FREAK',  'AKAZE' ,'SIFT' , 'LIDAR' ]

  keyAnalyse = {}

  framesHeader = ''
  maxFramesNumber = 1
  
  for idx, value in frames.items():
    framesHeader += ' Frame '
    framesHeader += idx
    framesHeader += ' ;'

    locIdx = int(idx)
    if maxFramesNumber < locIdx:
      maxFramesNumber = locIdx
    for actDet in det:
      if (actDet in frames[idx]):
        #frames[idx][actDet]['keyTimeSum'] = 0
        #frames[idx][actDet]['keyTimeCnt'] = 0
        if not (actDet in keyAnalyse):
          keyAnalyse[actDet]= {}
        #print (frames[idx][actDet])
        for actDesc in desc:  
          if(actDesc in frames[idx][actDet]):
           
            if not (actDesc in keyAnalyse[actDet]):
              keyAnalyse[actDet][actDesc] = {}
  
            keyAnalyse[actDet][actDesc][idx] = {}
            #print (actDet +' ' +actDesc + ' ' + str(idx) )
            if not ('CAM_TTC' in frames[idx][actDet][actDesc] ):
               keyAnalyse[actDet][actDesc][idx]['CAM_TTC']  = float('nan')
            else :
              keyAnalyse[actDet][actDesc][idx]['CAM_TTC'] =  frames[idx][actDet][actDesc]['CAM_TTC']
            #keyAnalyse[actDet][actDesc][idx]['LIDAR_TTC'] =  frames[idx][actDet][actDesc]['LIDAR_TTC']

          if actDesc == 'LIDAR':
            if not (actDesc in keyAnalyse[actDet]):
              keyAnalyse[actDet][actDesc] = {}
            keyAnalyse[actDet][actDesc][idx] = {}
            keyAnalyse[actDet][actDesc][idx]['CAM_TTC'] =  frames[idx][actDet][desc[0]]['LIDAR_TTC']


        
        

  ExtrectorTime = 'Detector ExtrectorTime;\n'
  ExtrectorKpC =  ';\n;\n Detector Extrector Key Point count;\n'
  #DescriptorTime = ' Descriptor Extrection Time'
  Match = ' TTC Time '
  
  #descriptorTime = ''
  CAM_TTC = ''
  LIDAR_TTC = ' '
  Header = 'Detector ; ' +  framesHeader  + ' \n'
  HeaderB = 'Descriptor ; ' +  framesHeader  + '\n'
  ExtrectorTime += Header
  ExtrectorKpC  += Header
  
  
  #print (keyAnalyse)

  for actDect , value in keyAnalyse.items():
    ExtrectorTime += actDect
    ExtrectorKpC +=actDect
    ExtrectorTime += ' '
    ExtrectorKpC += ' '
    
    #descriptorTime += ';\n;\n' + actDect + DescriptorTime +' ;\n' + HeaderB
    CAM_TTC += ';\n;\n' + actDect + Match +' ;\n' + HeaderB
    
    #analyse key points
    for actDesc in desc:
      if (actDesc in value ):
        #descriptorTime += actDesc + ' ; '
        CAM_TTC += actDesc + ' ; ;'
        for i in range (1,maxFramesNumber+1):
          locIDX = str(i)
          if locIDX in value[actDesc]:
            if  'CAM_TTC'in value[actDesc][locIDX]:
              CAM_TTC += str(value[actDesc][locIDX]['CAM_TTC']) + ' ; '
            else:
              CAM_TTC +=  ' ; '
        CAM_TTC += ';\n'
        #descriptorTime+= ';\n'

  analyseCVS+= CAM_TTC
  analyseCVS = analyseCVS.replace('.', ',')
  
  
  
  f = open("output.csv", "w")
  f.write(analyseCVS)
  f.close()
  #print (analyseCVS)
if __name__ == '__main__':
  main()