from cProfile import label
import datetime
import imp
from math import fabs
import os
import re
from tokenize import group
import matplotlib.pyplot as plt
from numpy import append
from scipy import size

class DataSturct:
    def __init__(self, x , y, type) :
        self.x = x
        self.y = y
        self.type = type


class SaveData:
    def __init__(self,fsm):
        self.all_data = dict()
        self.fsm = fsm
        self.nowTime =  datetime.datetime.now()
    
    def InsertData(self , data):
        for x,y in data.items():
            self.all_data[x] = y

        # print("now has data : " , self.all_data)


    def SaveToTxt(self):
        current_work_dir = os.path.dirname(__file__)
        path = str()
        if(self.fsm):
            path =  str(current_work_dir) + "/data/"  + str(self.nowTime)+ "_fsm" + ".txt"
        else:
            path =  str(current_work_dir)  + "/data/" + str(self.nowTime) + "_notfsm" + ".txt"

        fw = open(path ,  'w')

        for x,y in self.all_data.items():
            fw.writelines(['x:' , str(x) , '#' ,'y:' ,str(y) , '#'])
            fw.write("\n")

        fw.close()
        print(" Save txt to " + path)


    def TakeDataAndShow(self , paths):
        data_list = list()
        results = dict()
        i = 1
        for path in paths:
            current_work_dir = os.path.dirname(__file__) 
            f = open(str(current_work_dir) + "/data/" + str(path),"r")
            lines = f.readlines()
            result = dict()
            data = dict()
            type = str()
            if( re.search(r'notfsm', path)):
                type = "Q-Learning"
            else:
                type = "SCT+Q-Learning" 

            for line in lines:
                x = re.search(r'x:(\d+)', line)
                y = re.search(r'y:(\d+)', line)
                if x and y:
                    x = re.search(r'(\d+)', x.group(0))
                    y = re.search(r'(\d+)', y.group(0))

                    # result = dict{x : y}
                    x = int(x.group(0))
                    y = int(y.group(0))
                    result [x] = y



            # data = dict{ type : {x : y}}    
            data[type] = result

            # results = dict{ str(path) : dict{type : {x : y}}}    
            results["path" + str(i)] = data
            i += 1

        self.show_plot(results)


    def show_plot(self , results):
        plt.figure(figsize=(14,5))
        data_list = list()
        x =  list()
        y = list()
        for (_ , datas) in results.items():
            # data_type = str: type
            data_type = list(datas.keys())[0]
            # data = { x : y }
            data = list(datas.values())
            # x
            x = list(data[0].keys())
            for ind in range(size(x)):
                x[ind]  = x[ind] *1
            # y
            y = list(data[0].values())
            for ind in range(size(y)):
                y[ind]  = y[ind] *1
        # for 
            # data_list.append(DataSturct(x,y,data_type) )

        # print(size(data_list))

        # res = list()
        # for n in  range(size(data_list)):
            # if(n+3 > size(data_list)):
                # break
        #     res .append (self.data_curve(data_list[n: n+3] , data_type))
        # print(size(res))
        

        # for da in res:
        #     x = da.x
        #     y = da.y
        #     data_type = da.type
            if data_type == "SCT+Q-Learning":
                plt.plot(x, y,label= data_type,linewidth= 2,marker='o')

            else:
                plt.plot(x, y,label= data_type,linewidth= 2,marker='*',linestyle='dotted')

        plt.legend()
        plt.xlabel('Episode')
        plt.ylabel('Steps-num')
        plt.title('Comparison of Q-Learnning and SCT/Q-learning for 1-convergence')

        plt.show()


    def data_curve(self,path,type):
        if(size(path)<1):
            return path
        step = 1
        res = list()
        if(size(path) == 1):
            t = 0.0
            while(t<1.0):
                res.append(path[0])
                t = t + step
            return res
        
        path1 = path[0 : size(path)-1]

        path2 = path[1 : size(path)]

        res1 = self.data_curve(path1)
        res2 = self.data_curve(path2)
        
        t = 0.0
        while(t<1):
            x = float((1.0 - t ) * res1[int(1.0/step*t)].x + t * res2[int((1.0 / step * t ))].y)
            y = float((1.0 - t ) * res1[int(1.0/step*t)].y + t * res2[int((1.0 / step * t) )].y)
            data = DataSturct(x,y,type)
            res.append(data)
            t = t + step
        
        return res




        
    

        

if __name__ == '__main__':
    MAKE_FAKE_DATA = False

    fw = SaveData(False)
    if(MAKE_FAKE_DATA):
        for n in range(100):
            fw.InsertData({n : n+20})
        fw.SaveToTxt()
    else:
        paths = ["2022-08-16 11:41:31.158919_notfsm.txt",
                        "2022-08-16 12:06:35.571205_fsm.txt"

                       
                ]
        fw.TakeDataAndShow(paths)
