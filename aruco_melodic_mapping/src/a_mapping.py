#!/usr/bin/python

import rospy
import numpy as np
import tf 
from math import sqrt
from aruco_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

false_id_marker =[177] 
markers_detected = [] 
aver_markers_pos = {} 
static = True

dic_prop = [("id",-1), 
("posx",0.0), 
("posy",0.0), 
("posz",0.0), 
("quatx",0.0), 
("quatx",0.0), 
("quaty",0.0), 
("quatz",0.0), 
("quatw",0.0), 
("visible", False),
("parent_frame", " "),
("child_frame", " "),
("count", 0),
("flag", False)
]

camera_position = [{
"id":-1,
"posx":0.0, 
"posy":0.0, 
"posz":0.0, 
"quatx":0.0, 
"quatx":0.0, 
"quaty":0.0, 
"quatz":0.0, 
"quatw":0.0,
"parent_frame":"/map ",
"child_frame": "/camera",
"count":1
}]

marker_counter = 0
marker_size = 0.085 
first_marker_detected = False 
pos_last_calc = 0
last_detected =[]
global_cont = 0
pos_tol = 0.002
eu_tol = 0.02


def pos_calc(child, c, parent,p): 
	
	if child[c]["child_frame"] == "/camera":
		flagchild = 1
		flagparent = 0

	else: 
		
		flagchild = 0
		flagparent = 0
	
	matrixchild = matrix_tf(child,c,flagchild)
	matrixparent = matrix_tf(parent,p,flagparent)
	matrix = np.dot(matrixparent, matrixchild)
	transl, quat = quat_trans_from_matrix(matrix)
	child[c]["posx"] = transl[0]
	child[c]["posy"] = transl[1]
	child[c]["posz"] = transl[2]
	child[c]["quatx"] = quat[0]
	child[c]["quaty"] = quat[1]
	child[c]["quatz"] = quat[2]
	child[c]["quatw"] = quat[3]

	return child


def sum_vect(vect, i, aruco1, index):
	if aruco1 == 0:

		aruco1 = sqrt((vect[i].pose.pose.position.x**2)+(vect[i].pose.pose.position.y**2)+(vect[i].pose.pose.position.z**2))
		index = i
		return index,aruco1

	else:

		aruco2 = sqrt((vect[i].pose.pose.position.x**2)+(vect[i].pose.pose.position.y**2)+(vect[i].pose.pose.position.z**2))

		if aruco2 < aruco1:

			aruco1 = aruco2
			index = i
			return index, aruco1
		
		else:
			return index,aruco1


def matrix_tf(data, i, flag): 
	

	tl = tf.TransformerROS()
	px=data[i]["posx"]
	py=data[i]["posy"]
	pz=data[i]["posz"]
	rx=data[i]["quatx"]
	ry=data[i]["quaty"]
	rz=data[i]["quatz"]
	w=data[i]["quatw"]

	matrix=np.array(tl.fromTranslationRotation((px,py,pz), (rx,ry,rz,w))) 

	if flag:

		matrixrotinver = np.linalg.inv(matrix[0:3,0:3])
		transinver = np.dot(matrixrotinver,-matrix[0:3,3]) 
		matrixinver = np.insert(matrixrotinver,matrixrotinver.shape[1],transinver,1)
		matrixinver = np.insert(matrixinver,matrixinver.shape[0],np.array([0,0,0,1]),0)

		return matrixinver
	else:
		return matrix


def quat_trans_from_matrix(matrix): 
		quat = tf.transformations.quaternion_from_matrix(matrix)
		translation = tf.transformations.translation_from_matrix(matrix)

		return translation, quat


def del_false_id(array):

	global false_id_marker

	for i in array:
		if false_id_marker.count(i.id) > 0:
			array.remove(i)
	
	return array
		

def tolerance(array, index, div):

	global pos_tol, eu_tol

	dic_results = {num: {} for num in range(len(array[index]))}
	results = {num: 0 for num in range(div)}


	for i in range(len(array[index])):

		for y in range(len(array[index][i])-1):

			x = y + 1

			while x < len(array[index][i]):

				r = abs(array[index][i][y] - array[index][i][x])

				if i <= 3: 

					if r <= pos_tol:

						results[x] = results[x] + 1
						results[y] = results[y] + 1
						

				else: 

					if r <= eu_tol:

						results[x] = results[x] + 1
						results[y] = results[y] + 1
	
				x += 1
		
		dic_results[i] = results
		results = {num: 0 for num in range(div)}

	return dic_results


def quat_euler(data, index,flag):
	
	if flag == False: 

		qx = data[index]["quatx"]
		qy = data[index]["quaty"]
		qz = data[index]["quatz"]
		qw = data[index]["quatw"]

		euler = tf.transformations.euler_from_quaternion([qx,qy,qz,qw])

		return euler

	else: 

		roll = (sum(data[index][3]))/len(data[index][3])
		pitch = (sum(data[index][4]))/len(data[index][4])
		yaw = (sum(data[index][5]))/len(data[index][5])

		quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

		return quat



def pass_info(new_data, old_data, inew, iold): 
	if new_data[inew]["child_frame"] != "/camera":
		new_data[inew]["id"] = old_data[iold].id

	new_data[inew]["posx"] = old_data[iold].pose.pose.position.x
	new_data[inew]["posy"] = old_data[iold].pose.pose.position.y
	new_data[inew]["posz"] = old_data[iold].pose.pose.position.z
	new_data[inew]["quatx"] = old_data[iold].pose.pose.orientation.x
	new_data[inew]["quaty"] = old_data[iold].pose.pose.orientation.y
	new_data[inew]["quatz"] = old_data[iold].pose.pose.orientation.z
	new_data[inew]["quatw"] = old_data[iold].pose.pose.orientation.w

	return new_data


def sum_rest(a, b): 

	if (a > 0 and b > 0) or (a < 0 and b < 0):
		result = a - b
	else:
		result = a + b
	
	return result



def average(av_data, av_id, data, data_id, div, delete):
				
	av_data[av_id][0].append(data[data_id]["posx"])
	av_data[av_id][1].append(data[data_id]["posy"])
	av_data[av_id][2].append(data[data_id]["posz"])

	euler = quat_euler(data,data_id,False)

	av_data[av_id][3].append(euler[0])
	av_data[av_id][4].append(euler[1])
	av_data[av_id][5].append(euler[2])

	if data[data_id]["count"] == div: 

		result = tolerance(av_data, av_id, div)
		cont = 0 
		flag = True
		c = 0

		for i in result.values():

			if sum(i.values()) == 0: 

				delete.append(data_id)
				flag = False

				break

			else:

				val = list(i.values())
				key = list(i.keys())
				key = key[::-1]

				for y in key:

					if val[y] == 0:

						del av_data[av_id][cont][y]
			cont += 1

		if flag:

			data[data_id]["posx"] = (sum(av_data[av_id][0]))/len(av_data[av_id][0]) 
			data[data_id]["posy"] = (sum(av_data[av_id][1]))/len(av_data[av_id][1])
			data[data_id]["posz"] = (sum(av_data[av_id][2]))/len(av_data[av_id][2])

			quat = quat_euler(av_data,av_id,True) 

			data[data_id]["quatx"] = quat[0] 
			data[data_id]["quaty"] = quat[1]
			data[data_id]["quatz"] = quat[2]
			data[data_id]["quatw"] = quat[3]

			for i in range(len(av_data[av_id])):

				av_data[av_id][i][0] = (sum(av_data[av_id][i]))/len(av_data[av_id][i]) 

				del av_data[av_id][i][1::] 

			if data[data_id]["id"] != -1:

				data[data_id]["visible"] = True
				data[data_id]["flag"] = True
			
	return data, av_data, delete



def imageP(array): 

	global markers_detected, dic_prop, first_marker_detected, camera_position, marker_counter, pos_last_calc, last_detected, aver_markers_pos, global_cont, static
	
	marker_array = array.markers 
	pos_rep_mark = [] 
	visible_markers_before = {} 
	visible_markers_now = [] 
	visible_in_array =[] 
	to_calculate =[]
	near_aruco = 0 
	index = "N/A"
	distance = 1.5
	times_calc = 4
	to_del =[]
	global_cont += 1

	print(static)

	marker_array = del_false_id(marker_array)

	if first_marker_detected == False and len(marker_array) > 0 and sqrt((marker_array[0].pose.pose.position.x**2)+(marker_array[0].pose.pose.position.y**2)+(marker_array[0].pose.pose.position.z**2))< distance: 
		markers_detected.append(dict(dic_prop))
		markers_detected[0]["id"] = marker_array[0].id 
		markers_detected[0]["quatw"] = 1.0 
		markers_detected[0]["visible"] = True
		markers_detected[0]["parent_frame"] =  "/map"
		markers_detected[0]["child_frame"] = "marker_%d" %markers_detected[0]["id"]
		markers_detected[0]["count"] = times_calc
		markers_detected[0]["flag"] = True
		first_marker_detected = True 
		marker_counter += 1

	if pos_last_calc != "N/A":

		last_detected = [dict(markers_detected[i]) for i in range(len(markers_detected))] 

	
			
	for i in range(len(markers_detected)):

		visible = False

		for cont in range(len(marker_array)):

			if markers_detected[i]["id"] == marker_array[cont].id and sqrt((marker_array[cont].pose.pose.position.x**2)+(marker_array[cont].pose.pose.position.y**2)+(marker_array[cont].pose.pose.position.z**2))< distance: #encontro un marcador que ya se habia visto

				pos_rep_mark.append(cont)
				visible = True

				if markers_detected[i]["visible"] == False:	
					
					
					if pos_last_calc != "N/A":

						if markers_detected[i]["id"]!= markers_detected[0]["id"]: 
				
							markers_detected = pass_info(markers_detected,marker_array,i,cont)
							visible_markers_now.append(i) 
							markers_detected[i]["count"] += 1
							to_calculate.append(i)

						else:
							
							visible_markers_before[cont]=i 
							visible_in_array.append(cont)
							markers_detected[0]["visible"] = True
							markers_detected[0]["count"] = times_calc


					else:
						
						markers_detected[i]["visible"] = True
						markers_detected[i]["count"] = times_calc
						visible_markers_before[cont]=i 
						visible_in_array.append(cont) 		 

				else:

					visible_markers_before[cont]=i 
					visible_in_array.append(cont) 

				break
		
		if visible == False: 

			if markers_detected[i]["flag"] == False:

				to_del.append(i)

			elif static == False:

				markers_detected[i]["count"] = 1
				markers_detected[i]["visible"] = False

				if markers_detected[i]["id"] != markers_detected[0]["id"]:

					for x in range(len(aver_markers_pos[markers_detected[i]["id"]])):

						del aver_markers_pos[markers_detected[i]["id"]][x][1::] 


	if len(pos_rep_mark) != 0:

		for i in range(len(marker_array)): 
			
			if pos_rep_mark.count(i) == 0 and len(visible_in_array)!=0 and sqrt((marker_array[i].pose.pose.position.x**2)+(marker_array[i].pose.pose.position.y**2)+(marker_array[i].pose.pose.position.z**2))< distance: #si la posicion i esta en la lista es porque el id ya esta repetido y se continua con el siguiente

				markers_detected.append(dict(dic_prop)) 
				markers_detected = pass_info(markers_detected,marker_array,-1, i)
				markers_detected[-1]["child_frame"] = "marker_%d" %markers_detected[-1]["id"]
				markers_detected[-1]["count"] = 1
				markers_detected[-1]["visible"] = False
				marker_counter += 1
				to_calculate.append(marker_counter-1)
				aver_markers_pos[markers_detected[-1]["id"]] = [[],[],[],[],[],[]] 

			if visible_in_array.count(i) > 0:  

				index, near_aruco = sum_vect(marker_array, i, near_aruco, index) 
	
	pos_last_calc = index 

	
	if index != "N/A": 

		camera_position = pass_info(camera_position, marker_array, 0, index)  		
		camera_position = pos_calc(camera_position, 0, markers_detected, visible_markers_before[index]) 
		camera_position[0]["count"] += 1

		camera_position[0]["count"] = 1
		camera_position[0]["parent_frame"] = "/map"

		publishMarker(camera_position, 0)
		publish_tf(camera_position, 0)

		for i in to_calculate: 

			markers_detected[i]["parent_frame"] = "/map"
			markers_detected = pos_calc(markers_detected, i, camera_position, 0)
			markers_detected, aver_markers_pos, to_del = average(aver_markers_pos, markers_detected[i]["id"], markers_detected, i, times_calc, to_del)


		markers_to_pub = list(visible_markers_before.values()) + to_calculate 

		for i in markers_to_pub: 
			if markers_detected[i]["visible"] == True:
				publishMarker(markers_detected, i)
				publish_tf(markers_detected, i)


		markers_detected = [dict(last_detected[i]) for i in range(len(last_detected))]

		for i in visible_markers_now:

			markers_detected[i]["visible"] = True 

	to_del.sort()
	to_del = to_del[::-1]

	for i in to_del:

		del aver_markers_pos[markers_detected[i]["id"]]
		del markers_detected[i]
		del last_detected[i]
		marker_counter -= 1
	

	
def publishMarker(publisher, index):
	
	global marker_size
	
	pub=rospy.Publisher("/aruco/pub_marker", Marker, queue_size=100)

	viz_marker = Marker()

	viz_marker.header.frame_id = "/map" 

	viz_marker.header.stamp = rospy.Time.now()
	viz_marker.ns = "basic_shapes"
	viz_marker.id = publisher[index]["id"]
	viz_marker.type = viz_marker.CUBE
	viz_marker.action = viz_marker.ADD

	viz_marker.pose.position.x = publisher[index]["posx"]
	viz_marker.pose.position.y = publisher[index]["posy"]
	viz_marker.pose.position.z = publisher[index]["posz"]
		
	viz_marker.pose.orientation.x = publisher[index]["quatx"]
	viz_marker.pose.orientation.y = publisher[index]["quaty"]
	viz_marker.pose.orientation.z = publisher[index]["quatz"]
	viz_marker.pose.orientation.w = publisher[index]["quatw"]
		
	viz_marker.scale.x = marker_size
	viz_marker.scale.y = marker_size
	viz_marker.scale.z = 0.01

	viz_marker.color.r = 1
	viz_marker.color.g = 1
	viz_marker.color.b = 1
	viz_marker.color.a = 1

	pub.publish(viz_marker) 


def publish_tf(data, i): 

	tl = tf.TransformBroadcaster()

	px = data[i]["posx"]
	py = data[i]["posy"]
	pz = data[i]["posz"]
	rx = data[i]["quatx"]
	ry = data[i]["quaty"]
	rz = data[i]["quatz"]
	w = data[i]["quatw"]
	parent = "/map"
	child = data[i]["child_frame"]
	time = rospy.Time.now()

	tl.sendTransform((px,py,pz), (rx,ry,rz,w), time, child, parent)



def listener(): 
	global markers_detected, dic_prop, num_markers, cast

	rospy.init_node('a_mapping', anonymous=True) 

	rospy.Subscriber('/aruco/markers', MarkerArray, imageP) 

	rate = rospy.Rate(0.1) 
	while not rospy.is_shutdown():
		rate.sleep()


if __name__ == '__main__':

	try:
		listener()

	except rospy.ROSInterruptException:
		rospy.loginfo("ERROR")
