import os
from typing import List

def readFile(filePath:str) -> list:
    lines=[]
    if(os.path.exists(filePath)):
        logFile=open(filePath, 'r')
        lines = logFile.readlines()
        logFile.close()
    else:
        print("No  such file!: ",filePath)
    return lines

def parseLines(lines:List[str]) -> List[dict]:
    listedDict = []
    for line in lines:
        lineDict = {}
        csvList=line.strip().split(",")
        firstPt=csvList[0].replace("["," ").replace("]"," ").split(" ")
        for i in firstPt:
            if(i == ""):
                firstPt.remove(i)        
        lineDict["date"]=firstPt[0]
        lineDict["time"]=firstPt[1]
        lineDict["type"]=firstPt[2]
        csvList.pop(0)
        # csvList[0] = firstPt[3]
        
        for item in csvList:
            itemParts = item.split("=")
            lineDict[itemParts[0]] = itemParts[1]
        
        listedDict.append(lineDict)
        # print(lineDict)
        
    return listedDict
        
def parseLinesFromLog(logPath:str) -> List[dict]:
    return parseLines(readFile(logPath))


def parseGpsLines(lines:List[str]) -> List[dict]:
    listedDict = []
    for line in lines:
        if(line.strip() == ""):
            continue
        lineDict = {}
        csvList=line.strip().split(",")
        firstPt=csvList[0].replace("["," ").replace("]"," ").split(" ")
        for i in firstPt:
            if(i == ""):
                firstPt.remove(i)        
        lineDict["date"]=firstPt[0]
        lineDict["time"]=firstPt[1]
        lineDict["type"]=firstPt[2]
        csvList.pop(0)
        
        ## TODO: other gps parts should be added in here
        if(csvList[6] != ""):
            lineDict["speed"] = float(csvList[6]) * 1.852 # speed in kmh
        else:
            lineDict["speed"] = 0.0
        
        
        listedDict.append(lineDict)
        # print(lineDict)
        
    return listedDict

def parseGpsLinesFromLog(logPath:str) -> List[dict]:
    return parseGpsLines(readFile(logPath))


def getGpsEntryAtTime(gpsEntries:List[dict], timeStr:str) -> dict:
    reqTime = timeStr.split(".")[0]
    for ent in gpsEntries:
        entryTime = ent["time"].split(".")[0]
        if(entryTime == reqTime):
            return  ent
    return None

def main():
    listOfDict = parseLines(readFile("exampleAccelLog.txt"))
    
    axList=[]
    for i in listOfDict:
        print(i)
    
if __name__ == "__main__":
    main()
    