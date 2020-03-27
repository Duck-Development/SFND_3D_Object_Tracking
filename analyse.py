import sys
import os

def main():
  frames ={};
  analyseCVS = ''
  
  filepath = sys.argv[1];
  print ("Open File")
  print (filepath)
  detector = ""
  descripor = "" 
  with open(filepath) as fp:
    for line in fp:
      if line.startswith('*******************************'):
        frame = 0;
      
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
          frames[frame] = {};
      
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

    print( frames)    
    exit(0)

##full data parse ready
    det = [ "HARRIS" , "SHITOMASI" , "ORB" , "SIFT" , "FAST", "BRISK" , "AKAZE"  ];
    desc = [ "BRISK" ,  "BRIEF" ,  "ORB" , "FREAK",  "AKAZE" ,"SIFT"  ];

  keyAnalyse = {}
  
  for idx, value in frames.items():
    for actDet in det:
      if (actDet in frames[idx]):
        frames[idx][actDet]['keyTimeSum'] = 0
        frames[idx][actDet]['keyTimeCnt'] = 0
        if not (actDet in keyAnalyse):
          keyAnalyse[actDet]= {}
        #print (frames[idx][actDet])
        for actDesc in desc:  
          if(actDesc in frames[idx][actDet]):
            #print(actDesc)
            frames[idx][actDet]['keyTimeCnt'] += 1
            frames[idx][actDet]['keyTimeSum'] += frames[idx][actDet][actDesc]['keyPointTime']
            if not ('keyPointCnt' in frames[idx][actDet]):
              frames[idx][actDet]['keyPointCnt']= frames[idx][actDet][actDesc]['keyPointCnt']
            
            if not (actDesc in keyAnalyse[actDet]):
              keyAnalyse[actDet][actDesc] = {}
  
            keyAnalyse[actDet][actDesc][idx] = {}
            #print (actDet +' ' +actDesc + ' ' + str(idx) )
            keyAnalyse[actDet][actDesc][idx]['descriporTime'] =  frames[idx][actDet][actDesc]['descriporTime']
            if ( 'match' in frames[idx][actDet][actDesc]):
              keyAnalyse[actDet][actDesc][idx]['match'] =  frames[idx][actDet][actDesc]['match']


        if frames[idx][actDet]['keyTimeCnt'] > 0:
          frames[idx][actDet]['keyPointTimeAWG'] = str(frames[idx][actDet]['keyTimeSum']/float(frames[idx][actDet]['keyTimeCnt']))
        else:
          frames[idx][actDet]['keyPointTimeAWG'] = ' '
          frames[idx][actDet]['keyPointCnt'] = str(0)
        #if not (actDet in keyAnalyse):
          #keyAnalyse[actDet]= {}
        if not (idx in keyAnalyse[actDet]):
          keyAnalyse[actDet][idx] = {}
        keyAnalyse[actDet][idx]['keyPointTimeAWG'] = frames[idx][actDet]['keyPointTimeAWG'];
        keyAnalyse[actDet][idx]['keyPointCnt'] = frames[idx][actDet]['keyPointCnt'];
        
        

  ExtrectorTime = 'Detector ExtrectorTime;\n'
  ExtrectorKpC =  ';\n;\n Detector Extrector Key Point count;\n'
  DescriptorTime = ' Descriptor Extrection Time'
  Match = ' Matched KeyPoint'
  
  descriptorTime = ''
  match = ''
  Header = 'Detector ; Frame 1 ; Frame 2 ; Frame 3 ; Frame 4; Frame 5 ; Frame 6 ; Frame 7 ;Frame 8 ; Frame 9 ; Frame 10;\n'
  HeaderB = 'Descriptor ; Frame 1 ; Frame 2 ; Frame 3 ; Frame 4; Frame 5 ; Frame 6 ; Frame 7 ;Frame 8 ; Frame 9 ; Frame 10;\n'
  ExtrectorTime += Header
  ExtrectorKpC  += Header
  
  
  #print (keyAnalyse)
  for actDect , value in keyAnalyse.items():
    ExtrectorTime += actDect
    ExtrectorKpC +=actDect
    ExtrectorTime += ' '
    ExtrectorKpC += ' '
    
    descriptorTime += ';\n;\n' + actDect + DescriptorTime +' ;\n' + HeaderB
    match += ';\n;\n' + actDect + Match +' ;\n' + HeaderB
    
    for i in range(1,11):
      #print (i)
      if (i in value):
        #print (i)
        ExtrectorTime += ' ; '
        ExtrectorTime += str(value[i]['keyPointTimeAWG'])
        ExtrectorKpC += ' ; '
        ExtrectorKpC += str(value[i]['keyPointCnt'])
      else:
        ExtrectorTime += ' ; '
        ExtrectorKpC += ' ; '
    ExtrectorTime += ' ;\n '
    ExtrectorKpC += ' ;\n '

    #analyse key points
    for actDesc in desc:
      if (actDesc in value ):
        descriptorTime += actDesc + ' ; '
        match += actDesc + ' ; '
        for i in range (1,11):
          if i in value[actDesc]:
            if  'match'in value[actDesc][i]:
              match += str(value[actDesc][i]['match']) + ' ; '
            else:
              match +=  ' ; '
            if  'descriporTime'in value[actDesc][i]:
              descriptorTime += str(value[actDesc][i]['descriporTime']) + ' ; '
            else:
              descriptorTime +=  ' ; '
        match += ';\n'
        descriptorTime+= ';\n'
  analyseCVS+= ExtrectorTime
  analyseCVS+= ExtrectorKpC
  analyseCVS+= descriptorTime
  analyseCVS+= match
  analyseCVS = analyseCVS.replace('.', ',')
  
  
  
  f = open("output.csv", "w")
  f.write(analyseCVS)
  f.close()
  #print (analyseCVS)
if __name__ == '__main__':
  main();
