import copy
import numpy as np
import operator
import random
import math

class Compact(object):
	def __init__(self, id2window, capacity):
		self._id2window = id2window
		self._modified_id2window= copy.deepcopy(id2window)
		self._m = capacity
		self._table = {}
		self._compact_section = {}
		self._output = {}

	def init_table(self):
		for key, values in self._modified_id2window.iteritems():
			if values[0] in self._table:
				self._table[values[0]].append([key,values])
			else:
				self._table[values[0]] = [[key,values]]


	def create_compact(self):
		self._compact_section = {}
		checked = []
		for key, tasks in self._table.iteritems():
			if key not in checked:
				count = 0
				checked.append(key)
		 		task_num = len(self._table[key])
		 		self._compact_section[key] = tasks
		 		count += 1
		 		while task_num > self._m * count:
		 			if key+count in self._table and key+count not in checked:
		 				checked.append(key+count)
		 				self._compact_section[key] += self._table[key+count]
		 				task_num += len(self._table[key+count])
		 			elif key+count in self._table and key+count in checked:
		 				self._compact_section[key] += self._compact_section[key+count]
		 				task_num += len(self._compact_section[key+count])
		 				del self._compact_section[key+count]
		 			count += 1

		
	def offline_minimum(self):

		for key, compact_section in self._compact_section.iteritems():
			w = len(compact_section)
			for task in compact_section:
				task[1][0]= (task[1][0]-key)*self._m
				task[1][1] = min((task[1][1]- key)*self._m,w)
			compact_section.sort(key=lambda x: x[1][1])
		
		for key, compact_section in self._compact_section.iteritems():
			print compact_section
			w = len(compact_section)
			solution_set = set()
			k = range(w/self._m+1)
			ki = 0
			for min_index in range(w):
				if ki < len(k):
					if min_index == k[ki]*self._m:
						for ei,element in enumerate(compact_section):
							if element[1][0]/self._m == k[ki]:
								solution_set.add(ei)
						ki += 1

				print min_index, solution_set

				i = min(solution_set)
				solution_set.remove(i)
				self._output[compact_section[i][0]] = key - 1 + (min_index/self._m+1)
				if self._output[compact_section[i][0]] > self._id2window[compact_section[i][0]][1]:
					print "No Feasible Solution"
					return

		print len(self._output)
		for key in self._output:
			print key, self._output[key], self._id2window[key]

	

def main():
	capacity = 2
	id2window = {}
	for i in range(300):
		start = random.randint(0,120)
		window_len = random.randint(0,120)
		id2window[str(i)] =np.array([start, start+window_len])
	
	# id2window['1'] = np.array([0,1])
	# id2window['2'] = np.array([0,2])
	# id2window['3'] = np.array([2,2])
	# id2window['4'] = np.array([1,2])
	# id2window['5'] = np.array([0,0])

	window_list = []
	for key in id2window:
		window_list.append(id2window[key])
	window_list = np.array(window_list)
	print window_list
	

	compact = Compact(id2window, capacity)
	compact.init_table()
	compact.create_compact()
	compact.offline_minimum()




if __name__ == "__main__":
    main()