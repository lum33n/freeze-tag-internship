import time
import multiprocessing
import math
import random
import pylab
from PIL import Image, ImageTk, ImageDraw, ImageFont
from functools import lru_cache
import tkinter as tk
from functools import partial


#global variables
WINDOW_SIZE = [600, 600]
rip = 0

#class
class Node:
	def __init__(self, x = 0, y = 0, color = (255,255,255)): #the parameters allows the node to be represented with graphics
		self.x = x
		self.y = y
		self.color = color
		self.voisins = []
		self.identifiant = None #useful for carte

	def link(self, other, pound): #warning, this is an oriented graph so this will allow passage only from this one to the other
		if not other in self.voisins:
			self.voisins.append([other, pound])

class Carte: #if nodes are link to nodes that aren't in the graph, display wont work
	def __init__(self):
		#create an empty graph
		self.node_list = []
		self.temp_value = {
			"max_x": None,
			"min_x": None,
			"max_y": None,
			"min_y": None,
		} #makes displaying easier by calculating these when we add nodes and not with each display

	def copy(self):
		res = Carte()
		res.load(self.save())
		return res #hard to do better because of the way neighbour of a node is stored

	def random_graph(self, node_number, p_liaison): #the visual may be REALLY bad, only useful for complexity testing
		for i in range(node_number):
			new_node = Node(i*50, 0, color = (255,255,255))
			self.add_node(new_node)
			for j in range(i - 1):
				if random.random() < p_liaison:
					new_node.link(self.node_list[j], random.randint(1, 10))
				if random.random() < p_liaison:
					self.node_list[j].link(new_node, random.randint(1, 10))

	def add_node(self, node):
		#add the given node to the graph
		node.identifiant = len(self.node_list) #it will simply makes the ID of each nodes incremental since the length of the list will increment each time
		self.node_list.append(node)
		if self.temp_value["max_x"] != None:
			self.temp_value["max_x"] = max(node.x, self.temp_value["max_x"])
			self.temp_value["max_y"] = max(node.y, self.temp_value["max_y"])
			self.temp_value["min_x"] = min(node.x, self.temp_value["min_x"])
			self.temp_value["min_y"] = min(node.y, self.temp_value["min_y"])
		else:
			self.temp_value["max_x"] = node.x
			self.temp_value["max_y"] = node.y
			self.temp_value["min_x"] = node.x
			self.temp_value["min_y"] = node.y

	def del_node(self, node_id):
		#this function takes the node_id because if we have the node, we have the node_id and it allow to quickly find it in the list
		for i in self.node_list:
			for g in range(len(i.voisins)-1, -1, -1):
				if i.voisins[g][0].identifiant == node_id:
					del i.voisins[g]
		del self.node_list[node_id]
		temp = self.node_list
		self.node_list = []
		self.temp_value = {
			"max_x": None,
			"min_x": None,
			"max_y": None,
			"min_y": None,
		}
		for i in temp:
			self.add_node(i)
		#doing this is pretty much to simplify the programming of the recalculation of temp_values even if it is in fact ugly, it does not modify too much the complexity anyway.

	def offset(self, x_offset = 0, y_offset = 0): #can be useful when combining two graph made with the editor since all the points are within the same space, we may want to not making them intersect
		for i in self.node_list:
			i.x += x_offset
			i.y += y_offset

	def reverser(self): 
		#return a Carte with all the path reversed. useful for applying dijkstra the other way
		new_carte = Carte()
		new_carte.temp_value = self.temp_value
		for node in self.node_list:
			new_carte.add_node(Node(node.x, node.y, node.color))
		for node in self.node_list:
			for other, pound in node.voisins:
				new_carte.node_list[other.identifiant].link(new_carte.node_list[node.identifiant], pound)
		return new_carte

	def display(self, image_size = WINDOW_SIZE, bg_color = (0,0,0), radius_per_node = 30, path_to_highlight = []): #return an image
		"""Return a Pillow image representing the graph for humans"""
		#path_to_highlight is a node_list (with IDs or directly node, can even be both in the same list)
		assert self.node_list != []
		path_to_highlight = [self.node_list[i] if type(i) == int else i for i in path_to_highlight] #this step allows more flexibility to the arguments for humans to not break everything
		font = ImageFont.truetype("arial.ttf", 14)
		#to make human more friendly with variables' names
		t_min_x = self.temp_value["min_x"]
		t_max_x = self.temp_value["max_x"]
		t_min_y = -self.temp_value["max_y"] #y way is reversed in all the program because we want y axis to be going upward
		t_max_y = -self.temp_value["min_y"]
		dx = t_max_x - t_min_x
		dy = t_max_y - t_min_y
		tppsp_x = dx/(image_size[0] - 50)  #true_pixel_per_screen_pixels on x_axis
		spptp_x = 1/tppsp_x #screen pixels per true pixels on x_axis
		tppsp_y = dy/(image_size[1] - 50)  #true_pixel_per_screen_pixels on y_axis
		spptp_y = 1/tppsp_y #screen pixels per true pixels on y_axis
		#to print_pixel, add 25 pixels for both axis this allow to make a margin (we already anticipated the -50 in the above lines)
		im = Image.new("RGB", (image_size[0], image_size[1]), bg_color)
		dr = ImageDraw.Draw(im)
		for node in self.node_list:
			for voisin in node.voisins:
				pound = voisin[1]
				voisin = voisin[0]
				l = ((voisin.x - node.x)**2 + (voisin.y - node.y)**2)**0.5
				ax = (voisin.x - node.x)/l
				ay = -(voisin.y - node.y)/l
				#first, we create links so that nodes will be above links at the end which gives a better render

				#if the other node has us as a neighbour we should not place the white line as it could overwrite a past link.
				if not node in [k[0] for k in self.node_list[voisin.identifiant].voisins]:
					dr.line(((node.x-t_min_x)*spptp_x + 25, (-node.y-t_min_y)*spptp_y + 25, (node.x-t_min_x)*spptp_x + 25 + ax*(l)*spptp_x, (-node.y-t_min_y)*spptp_y + 25 + ay*l*spptp_y), fill = (255 - bg_color[0], 255 - bg_color[1], 255 - bg_color[2]), width = 1)
				dr.line(((node.x-t_min_x)*spptp_x + 25, (-node.y-t_min_y)*spptp_y + 25, (node.x-t_min_x)*spptp_x + 25 + ax*l/2*spptp_x, (-node.y-t_min_y)*spptp_y + 25 + ay*l/2*spptp_y), fill = node.color, width = 3)
				dr.text(((node.x-t_min_x)*spptp_x + 25 + ax*l/4*spptp_x, (-node.y-t_min_y)*spptp_y + 25 + ay*l/4*spptp_y), str(pound), font = font, fill = (255 - bg_color[0], 255 - bg_color[1], 255 - bg_color[2]))
		#we then draw the nodes
		for node in self.node_list:
			dr.ellipse(((node.x-t_min_x)*spptp_x - radius_per_node/2 + 25, (-node.y-t_min_y)*spptp_y - radius_per_node/2 + 25, (node.x-t_min_x)*spptp_x + radius_per_node/2 + 25, (-node.y-t_min_y)*spptp_y + radius_per_node/2 + 25), fill = node.color)
		#now we highlight the path asked
		for i in range(len(path_to_highlight) - 1):
			dr.line(((path_to_highlight[i].x - t_min_x)*spptp_x + 25, (-path_to_highlight[i].y - t_min_y)*spptp_y + 25, (path_to_highlight[i+1].x - t_min_x)*spptp_x + 25, (- path_to_highlight[i+1].y - t_min_y)*spptp_y + 25), fill = (200, 130, 0), width = 8)
		#and then we write the IDs of the nodes for better clarity when debugging.
		for node in self.node_list:
			dr.text(((node.x - t_min_x)*spptp_x + 25, (-node.y - t_min_y)*spptp_y), str(node.identifiant), font = font, fill = (255 - bg_color[0], bg_color[0], 255 - bg_color[0]))
		return im

	def save(self): #return a string that can be converted back using the load function
		data_storage = []
		data_storage.append(";".join([str(self.temp_value[i]) for i in self.temp_value.keys()]))
		for node in self.node_list:
			data_storage.append(";".join([str(i) for i in [node.x, node.y, node.color[0], node.color[1], node.color[2], node.identifiant] + [str(g[0].identifiant) + ":" + str(g[1]) for g in node.voisins]]))
		return "&".join(data_storage)

	def load(self, string): #load the string given by save to reproduce the same network
		data_storage = string.split("&")
		temp_values = data_storage[0].split(";")
		self.temp_value["max_x"] = int(temp_values[0])
		self.temp_value["min_x"] = int(temp_values[1])
		self.temp_value["max_y"] = int(temp_values[2])
		self.temp_value["min_y"] = int(temp_values[3])
		node_string = [i.split(";") for i in data_storage[1:]]
		node_temp_list = []
		for s in node_string:
			node_temp_list.append(Node(x = int(s[0]), y = int(s[1]), color = (int(s[2]), int(s[3]), int(s[4]))))
			node_temp_list[-1].identifiant = int(s[5])
		for i in range(len(node_string)):
			for g in range(len(node_string[i]) - 6):
				node_id, pound = node_string[i][6 + g].split(":")
				node_temp_list[i].link(node_temp_list[int(node_id)], int(pound))
		self.node_list = node_temp_list

class Editor: #to use it, just create the object. If you want to get the result at the closing of the windows, just get self.carte after creating the object
	def __init__(self, base_carte = Carte(), radius_per_node = 30):
		self.carte = base_carte
		self.fen = tk.Tk()
		self.radius_per_node = radius_per_node
		self.fen.geometry(str(WINDOW_SIZE[0] + 100) + "x" + str(WINDOW_SIZE[1]))
		self.color = [255,0,0]
		self.pound = 1
		self.text_variables = {
			"R": tk.StringVar(self.fen, str(self.color[0])),
			"G": tk.StringVar(self.fen, str(self.color[1])),
			"B": tk.StringVar(self.fen, str(self.color[2])),
			"Pound": tk.StringVar(self.fen, str(self.pound)),
		}
		self.entries = {
			"R": tk.Entry(self.fen, textvariable = self.text_variables["R"], bg = "red"),
			"G": tk.Entry(self.fen, textvariable = self.text_variables["G"], bg = "green"),
			"B": tk.Entry(self.fen, textvariable = self.text_variables["B"], bg = "blue"),
			"Pound": tk.Entry(self.fen, textvariable = self.text_variables["Pound"], bg = "brown"),
		}
		self.mode = 0 #0 is for node, 1 is for link
		self.past_click = None
		self.im_tk = None
		self.can = tk.Canvas(self.fen, height = WINDOW_SIZE[1], width = WINDOW_SIZE[0])
		self.can.place(x = 0)

		def refresh_text_variables(self):
			self.color[0] = int(self.text_variables["R"].get())
			self.color[1] = int(self.text_variables["G"].get())
			self.color[2] = int(self.text_variables["B"].get())
			self.pound = int(self.text_variables["Pound"].get())

		self.refresh_text_variables_func = refresh_text_variables
		self.entries["R"].place(x= WINDOW_SIZE[0] + 15, width = 20, height = 20, y = WINDOW_SIZE[1]/4)
		self.entries["G"].place(x= WINDOW_SIZE[0] + 40, width = 20, height = 20, y = WINDOW_SIZE[1]/4)
		self.entries["B"].place(x= WINDOW_SIZE[0] + 65, width = 20, height = 20, y = WINDOW_SIZE[1]/4)
		self.entries["Pound"].place(x= WINDOW_SIZE[0] + 35, width = 30, height = 20, y = WINDOW_SIZE[1]/3)


		def mode_button_click(self):
			self.mode ^= 1
			self.mode_button.config(text = ["Node", "Link"][self.mode])

		self.mode_button = tk.Button(self.fen, text = "Node", command = partial(mode_button_click, self))
		self.mode_button.place(x = WINDOW_SIZE[0] + 15, width = 70, height = 50, y = WINDOW_SIZE[1]/8)

		def left_click(self, event): #left click to add
			self.refresh_text_variables_func(self)
			event.y = WINDOW_SIZE[1] - event.y
			if self.mode == 0:
				validate = True
				for i in self.carte.node_list:
					if ((i.x - event.x)**2 + (i.y - event.y)**2) < self.radius_per_node**2:
						validate = False
				if validate:
					self.carte.add_node(Node(event.x, event.y, color = tuple(self.color)))
					self.display()
			elif self.mode == 1:
				if self.past_click == None:
					for i in self.carte.node_list:
						if ((i.x - event.x)**2 + (i.y - event.y)**2) <= self.radius_per_node**2:
							self.past_click = i
							break
				else:
					new_click = -1
					for i in self.carte.node_list:
						if ((i.x - event.x)**2 + (i.y - event.y)**2) <= self.radius_per_node**2:
							new_click = i
							break
					if new_click == -1 or new_click == self.past_click:
						self.past_click = None
					else:
						self.past_click.link(new_click, self.pound)
						self.past_click = None
						self.display()



		def right_click(self, event): #right click to remove
			self.refresh_text_variables_func(self)
			event.y = WINDOW_SIZE[1] - event.y
			if self.mode == 0:
				to_delete = -1
				for i in range(len(self.carte.node_list)):
					if ((self.carte.node_list[i].x - event.x)**2 + (self.carte.node_list[i].y - event.y)**2) <= self.radius_per_node**2:
						to_delete = i
						break
				if to_delete != -1:
					self.carte.del_node(to_delete)
					self.display()
			elif self.mode == 1:
				if self.past_click == None:
					for i in self.carte.node_list:
						if ((i.x - event.x)**2 + (i.y - event.y)**2) <= self.radius_per_node**2:
							self.past_click = i
							break
				else:
					new_click = -1
					for i in self.carte.node_list:
						if ((i.x - event.x)**2 + (i.y - event.y)**2) <= self.radius_per_node**2:
							new_click = i
							break
					if new_click == -1 or new_click == self.past_click:
						self.past_click = None
					else:
						indice_to_delete = -1
						for i in range(len(self.past_click.voisins)):
							if self.past_click.voisins[i][0] == new_click:
								indice_to_delete = i
								break
						if indice_to_delete != -1:
							del self.past_click.voisins[indice_to_delete]
							self.past_click = None
							self.display()
						else:
							self.past_click = None


		self.can.bind("<Button-1>", partial(left_click, self))
		self.can.bind("<Button-3>", partial(right_click, self))
		self.display()
		self.fen.mainloop()

	def display(self, image_size = WINDOW_SIZE, bg_color = (0,0,0)): #is a variant of the first display func but.. adapted and instead of returning an image, it prints it
		radius_per_node = self.radius_per_node
		font = ImageFont.truetype("arial.ttf", 14)
		tppsp_x = 1  #true_pixel_per_screen_pixels on x_axis
		spptp_x = 1 #screen pixels per true pixels on x_axis
		tppsp_y = 1  #true_pixel_per_screen_pixels on y_axis
		spptp_y = 1 #screen pixels per true pixels on y_axis
		#to print_pixel, add 25 pixels for both axis this allow to make a margin
		im = Image.new("RGB", (image_size[0], image_size[1]), bg_color)
		dr = ImageDraw.Draw(im)
		for node in self.carte.node_list:
			for voisin in node.voisins:
				pound = voisin[1]
				voisin = voisin[0]
				l = ((voisin.x - node.x)**2 + (voisin.y - node.y)**2)**0.5
				ax = (voisin.x - node.x)/l
				ay = -(voisin.y - node.y)/l
				if not node in [k[0] for k in self.carte.node_list[voisin.identifiant].voisins]:
					dr.line(((node.x)*spptp_x , (WINDOW_SIZE[1]-node.y)*spptp_y, (node.x)*spptp_x + ax*(l)*spptp_x, (WINDOW_SIZE[1]-node.y)*spptp_y + ay*l*spptp_y), fill = (255 - bg_color[0], 255 - bg_color[1], 255 - bg_color[2]), width = 1)
				dr.line(((node.x)*spptp_x, (WINDOW_SIZE[1]-node.y)*spptp_y, (node.x)*spptp_x + ax*l/2*spptp_x, (WINDOW_SIZE[1]-node.y)*spptp_y + ay*l/2*spptp_y), fill = node.color, width = 3)
				dr.text(((node.x)*spptp_x + ax*l/4*spptp_x, (WINDOW_SIZE[1]-node.y)*spptp_y + ay*l/4*spptp_y), str(pound), font = font, fill = (255 - bg_color[0], 255 - bg_color[1], 255 - bg_color[2]))
		for node in self.carte.node_list:
			dr.ellipse(((node.x)*spptp_x - radius_per_node/2, (WINDOW_SIZE[1]-node.y)*spptp_y - radius_per_node/2, (node.x)*spptp_x + radius_per_node/2, (WINDOW_SIZE[1]-node.y)*spptp_y + radius_per_node/2), fill = node.color)
		self.im_tk = ImageTk.PhotoImage(im)
		self.can.create_image(WINDOW_SIZE[0]/2, WINDOW_SIZE[1]/2, image = self.im_tk)



def distance(x, y):
	return math.sqrt((x[0] - y[0])**2 + (x[1] - y[1])**2)

class convexset:
	def __init__(self, l, n):
		self.l = l
		self.n = n

	def __iter__(self):
		self.i = 0
		self.j = 1
		return self

	def __next__(self):
		if self.i < len(self.l):

			if self.l[self.i] == self.n:
				self.i += 1
				if self.i >= len(self.l):
					raise StopIteration
			if self.l[self.j%len(self.l)] == self.n:
				self.j += 1
				if self.j >= len(self.l):
					self.i += 1
					self.j = self.i+1
					if self.i >= len(self.l):
						raise StopIteration

			X = []
			Xb = []
			for i in range(self.i, self.j):
				if self.l[i%len(self.l)] != self.n:
					X.append(self.l[i%len(self.l)])
			for i in range(self.j, self.i + len(self.l)):
				if self.l[i%len(self.l)] != self.n:
					Xb.append(self.l[i%len(self.l)])

			self.j += 1
			if (self.j == len(self.l)):
				self.i += 1
				self.j = self.i+1
			return X, Xb
		else:
			raise StopIteration

class powerset: #also return the complementary and ignore the n value
	def __init__(self, l, n):
		self.l = l
		self.n = n

	def __iter__(self):
		self.i = 0
		return self

	def __next__(self):
		if self.i < 2**(len(self.l)-1):
			self.i += 1
			res = []
			res2 = []
			compteur = 0
			for i in range(len(self.l)):
				if (self.l[i] == self.n):
					continue
				if (self.i // (2**compteur))%2 == 1:
					res.append(self.l[i])
				else:
					res2.append(self.l[i])
				compteur += 1
			return res, res2
		else:
			raise StopIteration

def caching(f):
	cache = {}
	def newf(*args):
		if str(args) in cache:
			return cache[str(args)]
		else:
			cache[str(args)] = f(*args)
			return cache[str(args)]
	return newf

def perf(f):
	def newf(*args):
		init = time.time()
		v = f(*args)
		print("executed in ", time.time() - init)
		return v
	return newf

class Graph:
	def __init__(self):
		self.point = [] #contains float couples

	def genrandomconvex(self, n: int, x = -50, y = 0, max_r = 100):
		assert n > 2
		X = [random.random() for i in range(n)]
		Y = [random.random() for i in range(n)]
		X.sort()
		Y.sort()
		shuffledx = X[1:-1]
		random.shuffle(shuffledx)
		shuffledy = Y[1:-1]
		random.shuffle(shuffledy)
		vectorsx = []
		firstchain = shuffledx[:(n-2)//2]
		secondchain = shuffledx[(n-2)//2:]
		vectorsx.append((firstchain[0] - X[0]))
		vectorsx.append((X[0] - secondchain[0]))
		vectorsx.append((X[-1] - firstchain[-1]))
		vectorsx.append((secondchain[-1] - X[-1]))
		for p in range(1, len(firstchain)):
			vectorsx.append((firstchain[p] - firstchain[p-1]))
		for p in range(1, len(secondchain)):
			vectorsx.append((secondchain[p-1] - secondchain[p]))
		vectorsy = []
		firstchain = shuffledy[:(n-2)//2]
		secondchain = shuffledy[(n-2)//2:]
		vectorsy.append((firstchain[0] - Y[0]))
		vectorsy.append((Y[0] - secondchain[0]))
		vectorsy.append((Y[-1] - firstchain[-1]))
		vectorsy.append((secondchain[-1] - Y[-1]))
		for p in range(1, len(firstchain)):
			vectorsy.append((firstchain[p] - firstchain[p-1]))
		for p in range(1, len(secondchain)):
			vectorsy.append((secondchain[p-1] - secondchain[p]))
		vectors = []
		random.shuffle(vectorsy)
		for yc in range(len(vectorsy)):
			vectors.append((vectorsx[yc], vectorsy[yc]))
		vectors.sort(key = lambda r: math.atan2(r[0], r[1]))
		self.point = [(x, y)]
		for (a, b) in vectors[:-1]:
			self.point.append((a + self.point[-1][0], b + self.point[-1][1]))

	def show(self, aretes = [], beg = None, alpha_angle = None, circle = False):
		pylab.clf()
		if (alpha_angle):
			pylab.plot([-math.cos(alpha_angle), math.cos(alpha_angle)], [-math.sin(alpha_angle), math.sin(alpha_angle)], color = "k")
			pylab.plot([-math.cos(alpha_angle), math.cos(alpha_angle)], [math.sin(alpha_angle), -math.sin(alpha_angle)], color = "k")
			pylab.plot([-math.cos(math.pi/2 - alpha_angle), math.cos(math.pi/2 - alpha_angle)], [-math.sin(math.pi/2 - alpha_angle), math.sin(math.pi/2 - alpha_angle)], color = "k")
			pylab.plot([-math.cos(math.pi/2 - alpha_angle), math.cos(math.pi/2 - alpha_angle)], [math.sin(math.pi/2 - alpha_angle), -math.sin(math.pi/2 - alpha_angle)], color = "k")
		if circle:
			circle1 = pylab.Circle((0, 0), 1, facecolor = "w", edgecolor='k')
			pylab.gcf().gca().add_patch(circle1)
		if len(aretes) == 0:
			pylab.scatter([x[0] for x in self.point], [y[1] for y in self.point], marker = "*", s = 80)
		else:
			for p in self.point:
				if p != beg:
					pylab.plot([p[0]], [p[1]], marker = "*")
			if beg:
				pylab.scatter([beg[0]], [beg[1]], marker = "X", s = 100)
			for arr in aretes:
				pylab.plot([arr[0][0], arr[1][0]], [arr[0][1], arr[1][1]])
		pylab.show()

	def brute_force(self):

		@caching
		def recursive_constructor(ens, beg):
			if len(ens) == 0 or ens == [beg]:
				return 0, []
			if len(ens) == 2:
				return distance(ens[0], ens[1]), [(ens[0], ens[1])]
			res = float("inf")
			minarr1 = -1
			minarr2 = -1
			minarr = None
			minX = None
			minXb = None
			for X, Xb in powerset(ens, beg):
				minind1 = -1
				minarr11 = None
				res1 = float("inf")
				for i in range(len(X)):
					temp, arr1 = recursive_constructor(X, X[i])
					temp += distance(X[i], beg)
					if temp < res1:
						res1 = temp
						minind1 = i
						minarr11 = arr1
				res2 = float("inf")
				minind2 = -1
				minarr22 = None
				for i in range(len(Xb)):
					temp, arr2 = recursive_constructor(Xb, Xb[i])
					temp += distance(Xb[i], beg)
					if temp < res2:
						res2 = temp
						minind2 = i
						minarr22 = arr2
				if max(res1, res2) < res:
					res = max(res1, res2)
					minarr1 = minind1
					minarr2 = minind2
					minarr = minarr11 + minarr22
					minX = X
					minXb = Xb

			minarr.append((beg, minX[minarr1]))
			minarr.append((beg, minXb[minarr2]))
			return res, minarr
			

		res = float("inf")
		minarr = None
		beg = None
		for i in range(len(self.point)):
			res1, aretes = recursive_constructor(self.point, self.point[i])
			if res1 < res:
				res = res1
				minarr = aretes
				beg = i
		return res, minarr, self.point[beg]

	def monalgo(self):
		@caching
		def recursive_constructor(ens, beg):
			global rip
			rip += 1
			if len(ens) == 0 or ens == [beg]:
				return 0, []
			if len(ens) == 2:
				return distance(ens[0], ens[1]), [(ens[0], ens[1])]
			res = float("inf")
			minarr1 = -1
			minarr2 = -1
			minarr = None
			minX = None
			minXb = None
			for X, Xb in convexset(ens, beg):
				minind1 = -1
				minarr11 = None
				res1 = float("inf")
				for i in range(len(X)):
					temp, arr1 = recursive_constructor(X, X[i])
					temp += distance(X[i], beg)
					if temp < res1:
						res1 = temp
						minind1 = i
						minarr11 = arr1
				res2 = float("inf")
				minind2 = -1
				minarr22 = None
				for i in range(len(Xb)):
					temp, arr2 = recursive_constructor(Xb, Xb[i])
					temp += distance(Xb[i], beg)
					if temp < res2:
						res2 = temp
						minind2 = i
						minarr22 = arr2
				if max(res1, res2) < res:
					res = max(res1, res2)
					minarr1 = minind1
					minarr2 = minind2
					minarr = minarr11 + minarr22
					minX = X
					minXb = Xb

			minarr.append((beg, minX[minarr1]))
			minarr.append((beg, minXb[minarr2]))
			return res, minarr
			

		res = float("inf")
		minarr = None
		beg = None
		for i in range(len(self.point)):
			res1, aretes = recursive_constructor(self.point, self.point[i])
			if res1 < res:
				res = res1
				minarr = aretes
				beg = i
		return res, minarr, self.point[beg]

	def monalgo_r(self):
		@caching
		def recursive_constructor(ens, beg):
			if len(ens) == 0 or ens == [beg]:
				return 0
			if len(ens) == 2:
				return distance(ens[0], ens[1])
			res = float("inf")
			for X, Xb in convexset(ens, beg):
				res1 = float("inf")
				for i in range(len(X)):
					temp = recursive_constructor(X, X[i])
					temp += distance(X[i], beg)
					if temp < res1:
						res1 = temp
				res2 = float("inf")
				for i in range(len(Xb)):
					temp = recursive_constructor(Xb, Xb[i])
					temp += distance(Xb[i], beg)
					if temp < res2:
						res2 = temp
				if max(res1, res2) < res:
					res = max(res1, res2)

			return res
			

		res = float("inf")
		for i in range(len(self.point)):
			res1 = recursive_constructor(self.point, self.point[i])
			if res1 < res:
				res = res1
		return res

def compare(n, p = False):
	G = Graph()
	G.genrandomconvex(n, x = -1, y = 0, max_r = 2)

	v1, arr1, beg1 = G.brute_force()
	v2, arr2, beg2 = G.monalgo()

	if p:
		G.show(arr1, beg1)
		G.show(arr2, beg2)
		print(v1, v2)
	test_arr = True
	for arr in arr1:
		if not arr in arr2 or (arr[1], arr[0]) in arr2:
			test_arr = False
			break
	if not test_arr and v1 == v2:
		print("Warning, different solutions, same result")
		print(G.point)
	if (v1 != v2):
		print("ALERTE ROUGE, SYSTEME CONTRE EXEMPLE DETECTE")
		print(v1, arr1, beg1)
		print(v2, arr2, beg2)
		print(G.point)
		return False
	return True

def carte_to_graph(carte):
	res = Graph()
	for node in carte.node_list:
		res.point.append((node.x, node.y))
	return res

def angles_to_graph(anglelist): #put all radius to 1 because for now my programs won't be useful for other values
	"""use degrees"""
	res = Graph()
	for angle in anglelist:
		res.point.append((math.cos(angle*math.pi/180), math.sin(angle*math.pi/180)))
	return res

#pastc = Carte()
#pastc.load("285;70;419;213&70;419;255;0;0;0&285;213;255;0;0;1")
#c = Editor(base_carte = pastc).carte
#print(c.save())
#working_graph = carte_to_graph(c)
#v, minarr, beg = working_graph.brute_force()
#print(v)
#print(carte_to_graph(pastc).brute_force())
#working_graph.show(minarr, beg)

#ys = [
#1, 1, 3, 2.73205, 3.82843, 3.35114, 3.73205, 3.43143, 3.61313, 3.41609, 3.52015, 3.38273, 3.44949, 3.34866, 3.45375, 3.31794, 3.4433, 3.33103, 3.42664, 3.33271, 3.40775, 3.32822, 3.38852, 3.32044, 3.36986, 3.31099, 3.36919, 3.3008, 3.36524, 3.30409, 3.35922, 3.3043, 3.35195, 3.30239, 3.33502, 3.29903, 3.31824, 3.29433, 3.30304, 3.28145, 3.28921, 3.26961, 3.27658, 3.25871, 3.265, 3.24864, 3.25434, 3.23931, 3.2445, 3.23064, 3.23539, 3.22257, 3.22693, 3.21504, 3.21906, 3.208, 3.21171, 3.2014, 3.20483, 3.19519, 3.2007, 3.18936, 3.20378, 3.18386, 3.20613, 3.18207, 3.20448, 3.18493, 3.20075, 3.18614, 3.19711, 3.18325, 3.19357, 3.18039, 3.19011, 3.17758, 3.18675, 3.17481, 3.18348, 3.1721, 3.1803, 3.16943, 3.17721, 3.16682, 3.1742, 3.16427, 3.17128, 3.16178, 3.16845, 3.15934, 3.16569, 3.15695, 3.16301, 3.15463, 3.16041, 3.15235, 3.15789, 3.15014, 3.15543, 3.14797, 3.1534, 3.14586, 3.15224, 3.1438, 3.15107, 3.14179, 3.14987, 3.14056, 3.14866, 3.13966, 3.14743, 3.13872, 3.1462, 3.13777, 3.14497, 3.1368, 3.14374, 3.13582, 3.1425, 3.13483, 3.14127, 3.13382, 3.14004, 3.13282, 3.13882, 3.1318, 3.1376, 3.13079, 3.13639, 3.12978, 3.13519, 3.12876, 3.134, 3.12775, 3.13283, 3.12674, 3.13166, 3.12574, 3.1305, 3.12474, 3.12936, 3.12375, 3.12823, 3.12276, 3.12661, 3.12178, 3.12493, 3.12081, 3.1233, 3.11985, 3.12171, 3.1189, 3.12016, 3.11795, 3.11865, 3.11657, 3.11717, 3.11515, 3.11574, 3.11376, 3.11433, 3.11241, 3.11296, 3.11109, 3.11163, 3.10979, 3.11032, 3.10853, 3.10905, 3.1073, 3.1078, 3.10609, 3.10658, 3.10491, 3.10539, 3.10376, 3.10423, 3.10263, 3.10309, 3.10153, 3.10197, 3.10045, 3.10088, 3.09939, 3.09982, 3.09835, 3.09877, 3.09734, 3.09775, 3.09634, 3.09674, 3.09537, 3.09576, 3.09442, 3.09517, 3.09348, 3.09487, 3.09256, 3.09456, 3.09167, 3.09424, 3.09078, 3.09392, 3.08992, 3.09359, 3.08964, 3.09325, 3.08938, 3.09292, 3.08911, 3.09257, 3.08883, 3.09223, 3.08855, 3.09188, 3.08827, 3.09153, 3.08798, 3.09117, 3.08768, 3.09082, 3.08739, 3.09046, 3.08708, 3.09013, 3.08678, 3.09001, 3.08647, 3.08988, 3.08616, 3.08974, 3.08597, 3.08959, 3.08588, 3.08944, 3.08578, 3.08927, 3.08567, 3.0891, 3.08556, 3.08893, 3.08544, 3.08874, 3.0853, 3.08856, 3.08517, 3.08836, 3.08502, 3.08816, 3.08487, 3.08775, 3.08472, 3.08715, 3.08455, 3.08657, 3.08439, 3.08599, 3.08422, 3.08541, 3.08377, 3.08485, 3.08323, 3.08429, 3.0827, 3.08374, 3.08217, 3.08319, 3.08165, 3.08266, 3.08113, 3.08213, 3.08062, 3.0816, 3.08012, 3.08108, 3.07962, 3.08057, 3.07913, 3.08007, 3.07865, 3.07957, 3.07817, 3.07908, 3.07769, 3.07859, 3.07723, 3.07811, 3.07676, 3.07763, 3.07631, 3.07716, 3.07586, 3.0767, 3.07541, 3.07624, 3.07497, 3.07579, 3.07453, 3.07534, 3.0741, ]
#
#ysnpair = [
#1, 3, 3.82843, 3.73205, 3.61313, 3.52015, 3.44949, 3.45375, 3.4433, 3.42664, 3.40775, 3.38852, 3.36986, 3.36919, 3.36524, 3.35922, 3.35195, 3.33502, 3.31824, 3.30304, 3.28921, 3.27658, 3.265, 3.25434, 3.2445, 3.23539, 3.22693, 3.21906, 3.21171, 3.20483, 3.2007, 3.20378, 3.20613, 3.20448, 3.20075, 3.19711, 3.19357, 3.19011, 3.18675, 3.18348, 3.1803, 3.17721, 3.1742, 3.17128, 3.16845, 3.16569, 3.16301, 3.16041, 3.15789, 3.15543, 3.1534, 3.15224, 3.15107, 3.14987, 3.14866, 3.14743, 3.1462, 3.14497, 3.14374, 3.1425, 3.14127, 3.14004, 3.13882, 3.1376, 3.13639, 3.13519, 3.134, 3.13283, 3.13166, 3.1305, 3.12936, 3.12823, 3.12661, 3.12493, 3.1233, 3.12171, 3.12016, 3.11865, 3.11717, 3.11574, 3.11433, 3.11296, 3.11163, 3.11032, 3.10905, 3.1078, 3.10658, 3.10539, 3.10423, 3.10309, 3.10197, 3.10088, 3.09982, 3.09877, 3.09775, 3.09674, 3.09576, 3.09517, 3.09487, 3.09456, 3.09424, 3.09392, 3.09359, 3.09325, 3.09292, 3.09257, 3.09223, 3.09188, 3.09153, 3.09117, 3.09082, 3.09046, 3.09013, 3.09001, 3.08988, 3.08974, 3.08959, 3.08944, 3.08927, 3.0891, 3.08893, 3.08874, 3.08856, 3.08836, 3.08816, 3.08775, 3.08715, 3.08657, 3.08599, 3.08541, 3.08485, 3.08429, 3.08374, 3.08319, 3.08266, 3.08213, 3.0816, 3.08108, 3.08057, 3.08007, 3.07957, 3.07908, 3.07859, 3.07811, 3.07763, 3.07716, 3.0767, 3.07624, 3.07579, 3.07534, 3.0749, 3.07446, 3.07403, 3.0736, 3.07318, 3.07285, 3.07282, 3.07278, 3.07274, 3.07269, 3.07264, 3.07259, 3.07253, 3.07246, 3.07239, 3.07232, 3.07225, 3.07217, 3.07209, 3.07201, 3.07192, 3.07183, 3.07172, 3.07132, 3.07091, 3.07052, 3.07012, 3.06973, 3.06935, 3.06897, 3.06859, 3.06822, 3.06785, 3.06749, 3.06713, 3.06677, 3.06642, 3.06607, 3.06572, 3.06538, 3.06504, 3.06471, 3.06438, 3.06405, 3.06372, 3.0634, 3.06308, 3.06277, 3.06246, 3.06215, 3.06184, 3.06154, 3.06124, 3.06094, 3.06065, 3.06036, 3.06007, 3.05978, 3.0595, 3.05922, 3.05894, 3.05867, 3.0584, 3.05813, 3.05786, 3.05765, 3.05746, 3.05726, 3.05707, 3.05688, 3.05668, 3.05649, 3.0563, 3.05612, 3.05593, 3.05574, 3.05575, 3.05581, 3.05586, 3.05591, 3.05596, 3.056, 3.05604, 3.05608, 3.05611, 3.05614, 3.05617, 3.0562, 3.0562, 3.05606, 3.05592, 3.05578, 3.05564, 3.05551, 3.05537, 3.05523, 3.05509, 3.05496, 3.05482, 3.05469, 3.05455, 3.05442, 3.05428, 3.05415, 3.05401, 3.05388, 3.05375, 3.05361, 3.05348, 3.05335, 3.05322, 3.05309, 3.05296, 3.05283, 3.0527, 3.05257, 3.05244, 3.05232, 3.05219, 3.05206, 3.05193, 3.05181, 3.05168, 3.05156, 3.05143, 3.05141, 3.05144, 3.05148, 3.05151, 3.05155, 3.05157, 3.0514, 3.05124, 3.05107, 3.05091, 3.05074, 3.05058, 3.05042, 3.05026, 3.0501, 3.04994, 3.04978, 3.04963, 3.04947, 3.04932, 3.04917, 3.04901, 3.04886, 3.04871, 3.04856, 3.04841, 3.04827, 3.04812, 3.04797, 3.04783, 3.04768, 3.04754, 3.0474, 3.04726, 3.04711, 3.04697, 3.04684, 3.0467, 3.04656, 3.04642, 3.04629, 3.04615, 3.04602, 3.04588, 3.04575, 3.04562, 3.04554, 3.04547, 3.0454, 3.04533, 3.04526, 3.04519, 3.04512, 3.04505, 3.04498, 3.04491, 3.04483, 3.04476, 3.04469, 3.04462, 3.04455, 3.04448, 3.04441, 3.04434, 3.04427, 3.04432, 3.04438, 3.04444, 3.0445, 3.04456, 3.04454, 3.04449, 3.04444, 3.04439, 3.04434, 3.0443, 3.04425, 3.04413, 3.044, 3.04388, 3.04376, 3.04364, 3.04352, 3.04339, 3.04328, 3.04316, 3.04304, 3.04292, 3.0428, 3.04269, 3.04257, 3.04246, 3.04234, 3.04223, 3.04212, 3.042, 3.04189, 3.04178, 3.04167, 3.04156, 3.04145, 3.04134, 3.04123, 3.04112, 3.04102, 3.04091, 3.0408, 3.0407, 3.04059, 3.04049, 3.04038, 3.04028, 3.04018, 3.04007, 3.03997, 3.03987, 3.03978, 3.03974, 3.0397, 3.03966, 3.03962, 3.03958, 3.03954, 3.0395, 3.03946, 3.03942, 3.03938, 3.03938, 3.03942, 3.03934, 3.03927, 3.03919, 3.03911, 3.03903, 3.03895, 3.03887, 3.0388, 3.03872, 3.03864, 3.03857, 3.03849, 3.03841, 3.03834, 3.03826, 3.03819, 3.03811, 3.03804, 3.03796, 3.03789, 3.03782, 3.03774, 3.03767, 3.0376, 3.03753, 3.03745, 3.03738, 3.03731, 3.03724, 3.03717, 3.0371, 3.03702, 3.03695, 3.03688, 3.03681, 3.03674, 3.03667, 3.03661, 3.03654, 3.03647, 3.0364, 3.03633, 3.03626, 3.0362, 3.03613, 3.03606, 3.03599, 3.03596, 3.03593, 3.03591, 3.03589, 3.03587, 3.03585, 3.03583, 3.03581, 3.03578, 3.03576, 3.03574, 3.03572, 3.0357, 3.03567, 3.03565, 3.03563, 3.0356, 3.03558, 3.03556, 3.03555, 3.03554, 3.03553, 3.03552, 3.03551, 3.0355, 3.03549, 3.03548, 3.03547, 3.03545, 3.03544, 3.03543, 3.03542, 3.03541, 3.03539, 3.03538, 3.03537, 3.03536, 3.03534, 3.03533, 3.03532, 3.0353, 3.03529, 3.03527, 3.03526, 3.03525, 3.03523, 3.03522, 3.0352, 3.03516, ]
#
#for i in range(1, len(ysnpair)-1):
#	if (ysnpair[i-1] >= ysnpair[i] and ysnpair[i] <= ysnpair[i+1]):
#		print("local minimum: ", 2*i)
#	if (ysnpair[i-1] <= ysnpair[i] and ysnpair[i] >= ysnpair[i+1]):
#		print("local maximum: ", 2*i)
#
##to do list: check for irregularities further away, calculate optimal values up to n = 26 if, differences are observed with segment, update so that it gives you the path. also, check the function for the particular tree of uniform observed for n <= 17 up to now if nothing is to be found or while the program is running. in particular, beware of the derative by i or by n.
#
#pylab.plot([2*i for i in range(len(ysnpair[2:]))], ysnpair[2:])
#pylab.show()

working_graph = angles_to_graph([0, 132, 172, 220, 310])
working_graph.show([((0, 0), working_graph.point[3]), (working_graph.point[3], working_graph.point[4]), ((working_graph.point[4]), (working_graph.point[0])), ((working_graph.point[3]), (working_graph.point[2])), ((working_graph.point[2]), (working_graph.point[1]))], 0,alpha_angle = 15*math.pi/180, circle = True)

#if __name__ == "__main__":
#	n = 11
#	test_per_n = 10
#	upper_n = 12
#	with multiprocessing.Pool(2) as p:
#		p.map(compare, [i//test_per_n + n for i in range((upper_n - n)*test_per_n)])

@perf
def timetest(n):
	G = Graph()
	G.genrandomconvex(n)
	v = G.monalgo()
	print(v)


#G = Graph()
#G.point = [
#(-0.373217, 0.00827304),
#(-0.52234, 0.553689),
#(-0.230526, 0.605496),
#(-0.194642, 0.600521),
#(-0.0154805, 0.566939),
#(0, -5.96046e-08),
#]
#G.point = [
#(-0.703987, -0.655238),
#(-0.802619, -0.695171),
#(-1.54137, -0.231741),
#(-1.42109, 0.298537),
#(-1.19221, 0.621058),
#(-0.736639, 0.572993),
#(1.78814e-07, 0),
#]
#G.point = [
#(-0.507372, -0.255478),
#(-1.0907, -0.121144),
#(-1.52136, 0.0427569),
#(-1.70525, 0.187054),
#(-1.77526, 0.850175),
#(-1.62607, 1.05445),
#(-1.12956, 0.871245),
#(-0.340738, 0.571718),
#(0, -5.96046e-08),
#]
#G.point = [
#(-0.514607,-0.737401),
#(-0.834748, -0.780882),
#(-1.32278, -0.550798),
#(-1.79945, -0.012),
#(-1.72848, 0.495632),
#(-1.55992, 0.807345),
#(-1.16199, 1.05807),
#(-1.06159, 1.05187),
#(-0.260453, 0.863837),
#(-0.100877, 0.545415),
#(-1.04308e-07, -1.19209e-07)]

#v, arr, beg = G.brute_force()
#v1, arr1, beg1 = G.monalgo()
#
#print(v, v1)
#
#G.show(arr, beg)
#G.show(arr1, beg1)
