# -*- coding: cp1251 -*-
'''������ Abaqus CAE ��� ���������� ����������� �� ������� ������ ������ � fe-safe'''
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
    �������� �������� ����������
    �������:
    par={'h1':0.0002,'h2':0.00004}
    set_values(part='Al',feature='Shell planar-1',par=par)
    '''
    p=model.parts[part] #������
    f=p.features[feature] #�������
    s=model.ConstrainedSketch(name='__edit__', objectToCopy=f.sketch) #���������� ����
    p.projectReferencesOntoSketch(filter=COPLANAR_EDGES, sketch=s, upToFeature=f) #������������
    for k,v in par.iteritems(): #��� ��� ���������
        s.parameters[k].setValues(expression=str(v)) #���������� ��������
    f.setValues(sketch=s) #���������� ����
    del s #�������
    p.regenerate() #������������ ������

def mesh_all(lst):
    '''���� ���� ��������� ��������
    lst - ������ ���� �������� ������'''
    ra=model.rootAssembly #������
    #�������� ������
    reg=[ra.instances[i] for i in lst]
    #reg=(ra.instances['Part-1-1'],ra.instances['Part-2-2'])
    ra.deleteMesh(regions=reg) #������� ����
    ra.generateMesh(regions=reg) #�������� ����

def JobSubmit(job):
    '''������ ������'''
    myJob = mdb.jobs[job] #������
    myJob.submit() #�������� ������
    # ������ ���� ������ �� ���� ����'�����
    myJob.waitForCompletion()

def readODB_set(set,step,var,pos=NODAL):
    '''������� ������ ���������� � ������ ������ ������� ��������� �����
    set - �������
    step - ����� �����
    var - �����:
    (('S', INTEGRATION_POINT, ((INVARIANT, 'Mises'), )), )
    (('U', NODAL, ((COMPONENT, 'U2'), )), )
    pos - �������: NODAL - ��� �����,INTEGRATION_POINT - ��� ��������
    �������: readODB_set(set='Set-1',step='Step-1',var=var)
    '''
    #�������� ���
    if pos==NODAL:    
        dat=session.xyDataListFromField(odb=myOdb, outputPosition=NODAL, variable=var, nodeSets=(set.upper(),)) #���
    if pos==INTEGRATION_POINT:
        dat=session.xyDataListFromField(odb=myOdb, outputPosition=INTEGRATION_POINT, variable=var, elementSets=(set.upper(),)) #���
    
    step_number=myOdb.steps[step].number #����� �����
    
    nframes=[] #������ ������� ������ � ������� �����
    for k in myOdb.steps.keys(): #��� ��� �����
        nframes.append(len(myOdb.steps[k].frames))
                      
    nstart_frame=nframes[step_number-2] #����� ����������� ������ �����
    nend_frame=int(sum(nframes[step_number-2:step_number])) #����� �������� ������ �����
    res=[] #������ ����������
    for x in dat: #��� ��� �����
        #x.data �� ((���,��������),(���,��������)...)
        res.append(x.data[nstart_frame:nend_frame]) #��� ����� � ��������� �����
                            
    #�������� �������� ���
    for k in session.xyDataObjects.keys():
        del session.xyDataObjects[k] 
    return res #�������� ������ �������

def readODB_set_(set,var):
    '''������� ������ ���������� � ������ ������ �������
    (��� ������ fe-safe)
    set - �������
    var - �����:
    (('LOGLife-Repeats', ELEMENT_NODAL), )
    (('FOS@Life=Infinite', ELEMENT_NODAL), )
    (('%%Failure@Life=5E6-Repeats', ELEMENT_NODAL), )
    �������: readODB_set_(set='Set-1',var=var)
    '''
    #�������� ���
    dat=session.xyDataListFromField(odb=myOdb, outputPosition=NODAL, variable=var, nodeSets=(set.upper(),)) #���
    
    res=[] #������ ����������
    for x in dat: #��� ��� �����
        #x.data �� ((���,��������),(���,��������)...)
        res.append(x.data) #���
                            
    #�������� �������� ���
    for k in session.xyDataObjects.keys():
        del session.xyDataObjects[k] 
    return res #�������� ������ �������

def readODB_set2(set,step,var,pos=NODAL):
    '''������� ������ ������� ���������� � ������ ������ ������� ��������� �����
    (���� ����������� ������������ readODB_set())
    set - �������
    step - ����
    var - �����:
    ('S','Mises')
    ('S','Pressure')
    ('U','Magnitude')
    ('U','U1')
    ('CPRESS','')
    ('D','') #���������� ������ ������ ������
    ('LOGLife-Repeats', '') # ???????
    ('FOS@Life=Infinite', '') # ???????
    ('%%Failure@Life=5E6-Repeats', '') # ???????
    pos - �������: NODAL - ��� �����,INTEGRATION_POINT - ��� ��������
    �������: readODB_set2(set='Cont',step='Step-1',var=('S','Mises'))
    '''
    if pos==NODAL:    
        s=myOdb.rootAssembly.nodeSets[set.upper()] #������� �����
    if pos==INTEGRATION_POINT:
        s=myOdb.rootAssembly.elementSets[set.upper()] #������� ��������
    m=[] #������ ������� ���������� � ��� ����� �������
    for f in myOdb.steps[step].frames: #��� ������� ������
        fo=f.fieldOutputs[var[0]].getSubset(region=s,position=pos) #���
        #openOdb(r'C:/Temp/Model-1.odb').steps['Step-1'].frames[4].fieldOutputs['CPRESS'].getSubset(position=NODAL, region=openOdb(r'C:/Temp/Model-1.odb').rootAssembly.nodeSets['CONT']).values[0].data
        res=[] #������ ����������
        for v in fo.values: #��� ������� �����/��������
            if var[1]=='Mises': res.append(v.mises)#������ �� ������ ����������
            if var[1]=='S11': res.append(v.data.tolist()[0])
            if var[1]=='S22': res.append(v.data.tolist()[1])
            if var[1]=='S33': res.append(v.data.tolist()[2])
            if var[1]=='S12': res.append(v.data.tolist()[3])
            if var[1]=='Pressure': res.append(v.press)
            if var[0]=='U' and var[1]=='Magnitude': res.append(v.magnitude)
            if var[1]=='U1': res.append(v.data.tolist()[0])
            if var[1]=='U2': res.append(v.data.tolist()[1])
            if var[0]=='CPRESS': res.append(v.data)
        m.append((f.frameValue, sum(res)/len(res)))  #������ ������ � ��� �����
    return m #�������� ������ �������

def findmax(data):
    '''������� ����������� �������� � ������ (���, ��������)'''
    max=(0,0)
    for x in data:
        if x[1]>max[1]:
            max=x
    return max

def runFeSafe(input_odb,input_stlx,output_odb):
    '''������ ����� ������ ������ � fe-safe
    input_odb - ����� �������� ����� ���������� Abaqus (��� ���������� .odb)
    input_stlx - ����� ����� ����� fe-safe (��� ���������� .stlx)
    output_odb - ����� ��������� ����� ���������� Abaqus (��� ���������� .odb)
    '''
    s=r'd:\Program Files\Safe_Technology\fe-safe\version.6.2\exe\fe-safe_cl.exe -s j=c:\1\{iodb}.odb b=c:\1\{istlx}.stlx o=c:\1\{oodb}.odb'
    s=s.format(iodb=input_odb, istlx=input_stlx, oodb=output_odb)
    # ������ ���������� � fe-safe �� ���� ����������
    subprocess.Popen(s).communicate()

def writeLDFfile(filename,lst):
    '''����� ���� ����� ���������� ������������ LDF fe-safe
    filename - ��� �����
    lst - ������ �����-�����
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

""" # ������ �����
    s=s.format(x=lst[0]) # ���� ����� � �������
    f.write(s)
    f.close()


import csv
csv_file=open("results.csv", "wb") #������� csv ����
writer = csv.writer(csv_file,delimiter = ';')
writer.writerow(['r','load','LogLife','FOS','%Failure']) #�������� �����
model=mdb.models['Model-3'] #������

for rad in [0.067,0.25,0.5,0.75,1.0]: #���� ��� ���� �������� ��������� r
    #���������� �������� ������������� ��������� r
    set_values(part='Part-1',feature='Solid revolve-1',par={'r':rad})
    #set_values(part='Part-1',feature='Shell planar-1',par={'r':rad})
    model.rootAssembly.regenerate() #�������� ������
    mesh_all(['Part-1-1']) #�������� ���� ��������� ��������
    #model.loads['Load-2'].setValues(magnitude=load) 
    JobSubmit('Job-3') #�������� ������

    for load in [0.1,0.15,0.2,0.25,0.3]: #���� ��� ���� �������� ������������ �����
        writeLDFfile('model3.ldf',[str(load)]) # ������ LDF ���� fe-safe
        oodb='results'+str(int(rad*1000))+'_'+str(int(load*100)) # ����� ���� ����� ����������
        runFeSafe('Job-3','model3',oodb) # �������� fe-safe
        
        #myOdb = openOdb(path=model.name + '.odb') #������� ���� ����� ����������
        myOdb = openOdb(path=oodb + '.odb') #������� ���� ����� ����������
        session.viewports['Viewport: 1'].setValues(displayedObject=myOdb)
        
        #�������� �������� ����������� � ������ ����� Set-1
        var=(('LOGLife-Repeats', ELEMENT_NODAL), )
        x1=readODB_set_(set='Set-1',var=var)
        x1min = min([x[0][1] for x in x1]) # ������ �������� ��������
        
        #�������� ���������� ������ � ������ ����� Set-1
        var=(('FOS@Life=Infinite', ELEMENT_NODAL), )
        x2=readODB_set_(set='Set-1',var=var)
        # x2=[((3.0, 1.62),), ((3.0, 2.0),), ...,((���,��������),)]
        x2min = min([x[0][1] for x in x2]) # ������ �������� ��������
        
        #�������� ������� ����� � ������ ����� Set-1
        var=(('%%Failure@Life=5E6-Repeats', ELEMENT_NODAL), )
        x3=readODB_set_(set='Set-1',var=var)
        x3max = max([x[0][1] for x in x3]) # ������ ����������� ��������
            
        '''
        #�������� ����������
        x1=readODB_set2(set='Up',step='Step-2',var=('S','S22'),pos=INTEGRATION_POINT)
        x1max=findmax(x1)#������ ����������� � ��� ������
        
        #�������� ���������� ����
        x2=readODB_set2(set='Nip',step='Step-1',var=('CPRESS',''))
        x2max=findmax(x2)#������ ����������� � ��� ������
        
        #�������� ����������� ����������
        #x3=readODB_set2(set='Bot',step='Step-2',var=('U','U2'))
        
        ##�������� ����������
        #var=(('S', INTEGRATION_POINT, ((INVARIANT, 'Mises'), )), )
        #print readODB_set(set='Up',step='Step-2',var=var)
        ##�������� ����������� ����������
        #var=(('U', NODAL, ((COMPONENT, 'U2'), )), )
        #print readODB_set(set='Bot',step='Step-2',var=var)
        '''
        #�������� ��� � ����
        writer.writerow([rad,load,x1min,x2min,x3max])
        myOdb.close() #������� ���� ����� ����������
        
csv_file.close() #������� ���� csv