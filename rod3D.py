# -*- coding: cp1251 -*-
'''Макрос Abaqus CAE для оптимізації конструкції за критерієм втомної міцності з fe-safe'''
from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *
import subprocess

def set_values(part,feature,par):
    '''
    Присвоює значення параметрам
    Приклад:
    par={'h1':0.0002,'h2':0.00004}
    set_values(part='Al',feature='Shell planar-1',par=par)
    '''
    p=model.parts[part] #деталь
    f=p.features[feature] #елемент
    s=model.ConstrainedSketch(name='__edit__', objectToCopy=f.sketch) #тимчасовий ескіз
    p.projectReferencesOntoSketch(filter=COPLANAR_EDGES, sketch=s, upToFeature=f) #спроектувати
    for k,v in par.iteritems(): #для всіх параметрів
        s.parameters[k].setValues(expression=str(v)) #установити значення
    f.setValues(sketch=s) #установити ескіз
    del s #знищити
    p.regenerate() #регенерувати деталь

def mesh_all(lst):
    '''Будує сітку скінченних елементів
    lst - список назв елементів зборки'''
    ra=model.rootAssembly #зборка
    #елементи зборки
    reg=[ra.instances[i] for i in lst]
    #reg=(ra.instances['Part-1-1'],ra.instances['Part-2-2'])
    ra.deleteMesh(regions=reg) #знищити сітку
    ra.generateMesh(regions=reg) #створити сітку

def JobSubmit(job):
    '''Виконує задачу'''
    myJob = mdb.jobs[job] #задача
    myJob.submit() #виконати задачу
    # Чекати поки задача не буде розв'язана
    myJob.waitForCompletion()

def readODB_set(set,step,var,pos=NODAL):
    '''Повертає список результатів в вузлах заданої множини вказаного кроку
    set - множина
    step - номер кроку
    var - змінна:
    (('S', INTEGRATION_POINT, ((INVARIANT, 'Mises'), )), )
    (('U', NODAL, ((COMPONENT, 'U2'), )), )
    pos - позиція: NODAL - для вузлів,INTEGRATION_POINT - для елементів
    Приклад: readODB_set(set='Set-1',step='Step-1',var=var)
    '''
    #отримати дані
    if pos==NODAL:    
        dat=session.xyDataListFromField(odb=myOdb, outputPosition=NODAL, variable=var, nodeSets=(set.upper(),)) #дані
    if pos==INTEGRATION_POINT:
        dat=session.xyDataListFromField(odb=myOdb, outputPosition=INTEGRATION_POINT, variable=var, elementSets=(set.upper(),)) #дані
    
    step_number=myOdb.steps[step].number #номер кроку
    
    nframes=[] #містить кількість фреймів в кожному кроці
    for k in myOdb.steps.keys(): #для всіх кроків
        nframes.append(len(myOdb.steps[k].frames))
                      
    nstart_frame=nframes[step_number-2] #номер початкового фрейма кроку
    nend_frame=int(sum(nframes[step_number-2:step_number])) #номер кінцевого фрейма кроку
    res=[] #список результатів
    for x in dat: #для всіх вузлів
        #x.data це ((час,значення),(час,значення)...)
        res.append(x.data[nstart_frame:nend_frame]) #дані тільки з вказаного кроку
                            
    #видалити тимчасові дані
    for k in session.xyDataObjects.keys():
        del session.xyDataObjects[k] 
    return res #повертае список значень

def readODB_set_(set,var):
    '''Повертає список результатів в вузлах заданої множини
    (для змінних fe-safe)
    set - множина
    var - змінна:
    (('LOGLife-Repeats', ELEMENT_NODAL), )
    (('FOS@Life=Infinite', ELEMENT_NODAL), )
    (('%%Failure@Life=5E6-Repeats', ELEMENT_NODAL), )
    Приклад: readODB_set_(set='Set-1',var=var)
    '''
    #отримати дані
    dat=session.xyDataListFromField(odb=myOdb, outputPosition=NODAL, variable=var, nodeSets=(set.upper(),)) #дані
    
    res=[] #список результатів
    for x in dat: #для всіх вузлів
        #x.data це ((час,значення),(час,значення)...)
        res.append(x.data) #дані
                            
    #видалити тимчасові дані
    for k in session.xyDataObjects.keys():
        del session.xyDataObjects[k] 
    return res #повертае список значень

def readODB_set2(set,step,var,pos=NODAL):
    '''Повертає список середніх результатів в вузлах заданої множини вказаного кроку
    (менш універсальна альтернатива readODB_set())
    set - множина
    step - крок
    var - змінна:
    ('S','Mises')
    ('S','Pressure')
    ('U','Magnitude')
    ('U','U1')
    ('CPRESS','')
    ('D','') #коефіцієнт запасу втомної міцності
    ('LOGLife-Repeats', '') # ???????
    ('FOS@Life=Infinite', '') # ???????
    ('%%Failure@Life=5E6-Repeats', '') # ???????
    pos - позиція: NODAL - для вузлів,INTEGRATION_POINT - для елементів
    Приклад: readODB_set2(set='Cont',step='Step-1',var=('S','Mises'))
    '''
    if pos==NODAL:    
        s=myOdb.rootAssembly.nodeSets[set.upper()] #множина вузлів
    if pos==INTEGRATION_POINT:
        s=myOdb.rootAssembly.elementSets[set.upper()] #множина елементів
    m=[] #список середніх результатів з усіх вузлів множини
    for f in myOdb.steps[step].frames: #для кожного фрейму
        fo=f.fieldOutputs[var[0]].getSubset(region=s,position=pos) #дані
        #openOdb(r'C:/Temp/Model-1.odb').steps['Step-1'].frames[4].fieldOutputs['CPRESS'].getSubset(position=NODAL, region=openOdb(r'C:/Temp/Model-1.odb').rootAssembly.nodeSets['CONT']).values[0].data
        res=[] #список результатів
        for v in fo.values: #для кожного вузла/елемента
            if var[1]=='Mises': res.append(v.mises)#додати до списку результатів
            if var[1]=='S11': res.append(v.data.tolist()[0])
            if var[1]=='S22': res.append(v.data.tolist()[1])
            if var[1]=='S33': res.append(v.data.tolist()[2])
            if var[1]=='S12': res.append(v.data.tolist()[3])
            if var[1]=='Pressure': res.append(v.press)
            if var[0]=='U' and var[1]=='Magnitude': res.append(v.magnitude)
            if var[1]=='U1': res.append(v.data.tolist()[0])
            if var[1]=='U2': res.append(v.data.tolist()[1])
            if var[0]=='CPRESS': res.append(v.data)
        m.append((f.frameValue, sum(res)/len(res)))  #додати середнє з усіх вузлів
    return m #повертае список значень

def findmax(data):
    '''Повертає максимальне значення в форматі (час, значення)'''
    max=(0,0)
    for x in data:
        if x[1]>max[1]:
            max=x
    return max

def runFeSafe(input_odb,input_stlx,output_odb):
    '''Виконує аналіз втомної міцності у fe-safe
    input_odb - назва вхідного файлу результатів Abaqus (без розширення .odb)
    input_stlx - назва файлу моделі fe-safe (без розширення .stlx)
    output_odb - назва вихідного файлу результатів Abaqus (без розширення .odb)
    '''
    s=r'd:\Program Files\Safe_Technology\fe-safe\version.6.2\exe\fe-safe_cl.exe -s j=c:\1\{iodb}.odb b=c:\1\{istlx}.stlx o=c:\1\{oodb}.odb'
    s=s.format(iodb=input_odb, istlx=input_stlx, oodb=output_odb)
    # виконує обчислення в fe-safe та чекає завершення
    subprocess.Popen(s).communicate()

def writeLDFfile(filename,lst):
    '''Змінює вміст файлу визначення навантаження LDF fe-safe
    filename - імя файлу
    lst - список рядків-даних
    '''
    f=open(filename, "w")
    s="""
# .ldf file created by fe-safe compliant product 6.2-01[mswin]

INIT
transitions=Yes
END

# Block number 1
BLOCK n=1, scale=1
lh=0 1 , ds=1, scale=1
lh=0 {x} , ds=2, scale=1
END

""" # шаблон файлу
    s=s.format(x=lst[0]) # вміст файлу з шаблону
    f.write(s)
    f.close()


import csv
csv_file=open("results.csv", "wb") #відкрити csv файл
writer = csv.writer(csv_file,delimiter = ';')
writer.writerow(['r','load','LogLife','FOS','%Failure']) #записати рядок
model=mdb.models['Model-3'] #модель

for rad in [0.067,0.25,0.5,0.75,1.0]: #цикл для зміни значення параметру r
    #установити значення геометричного параметру r
    set_values(part='Part-1',feature='Solid revolve-1',par={'r':rad})
    #set_values(part='Part-1',feature='Shell planar-1',par={'r':rad})
    model.rootAssembly.regenerate() #обновити модель
    mesh_all(['Part-1-1']) #створити сітку скінченних елементів
    #model.loads['Load-2'].setValues(magnitude=load) 
    JobSubmit('Job-3') #виконати задачу

    for load in [0.1,0.15,0.2,0.25,0.3]: #цикл для зміни значення навантаження згину
        writeLDFfile('model3.ldf',[str(load)]) # змінити LDF файл fe-safe
        oodb='results'+str(int(rad*1000))+'_'+str(int(load*100)) # назва бази даних результатів
        runFeSafe('Job-3','model3',oodb) # виконати fe-safe
        
        #myOdb = openOdb(path=model.name + '.odb') #відкрити базу даних результатів
        myOdb = openOdb(path=oodb + '.odb') #відкрити базу даних результатів
        session.viewports['Viewport: 1'].setValues(displayedObject=myOdb)
        
        #отримати логарифм довговічності у множині вузлів Set-1
        var=(('LOGLife-Repeats', ELEMENT_NODAL), )
        x1=readODB_set_(set='Set-1',var=var)
        x1min = min([x[0][1] for x in x1]) # знайти мінімальне значення
        
        #отримати коефіцієнт запасу у множині вузлів Set-1
        var=(('FOS@Life=Infinite', ELEMENT_NODAL), )
        x2=readODB_set_(set='Set-1',var=var)
        # x2=[((3.0, 1.62),), ((3.0, 2.0),), ...,((час,значення),)]
        x2min = min([x[0][1] for x in x2]) # знайти мінімальне значення
        
        #отримати відсоток відмов у множині вузлів Set-1
        var=(('%%Failure@Life=5E6-Repeats', ELEMENT_NODAL), )
        x3=readODB_set_(set='Set-1',var=var)
        x3max = max([x[0][1] for x in x3]) # знайти максимальне значення
            
        '''
        #отримати напруження
        x1=readODB_set2(set='Up',step='Step-2',var=('S','S22'),pos=INTEGRATION_POINT)
        x1max=findmax(x1)#знайти максимальне з усіх фреймів
        
        #отримати контактний тиск
        x2=readODB_set2(set='Nip',step='Step-1',var=('CPRESS',''))
        x2max=findmax(x2)#знайти максимальне з усіх фреймів
        
        #отримати вертикальне переміщення
        #x3=readODB_set2(set='Bot',step='Step-2',var=('U','U2'))
        
        ##отримати напруження
        #var=(('S', INTEGRATION_POINT, ((INVARIANT, 'Mises'), )), )
        #print readODB_set(set='Up',step='Step-2',var=var)
        ##отримати вертикальне переміщення
        #var=(('U', NODAL, ((COMPONENT, 'U2'), )), )
        #print readODB_set(set='Bot',step='Step-2',var=var)
        '''
        #записати дані у файл
        writer.writerow([rad,load,x1min,x2min,x3max])
        myOdb.close() #закрити базу даних результатів
        
csv_file.close() #закрити файл csv