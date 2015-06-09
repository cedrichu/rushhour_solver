import copy
import numpy as np
import operator

class SWO(object):
	def __init__(self, solver=None):
		self._solution = {}
		self._priority_sequence = []
		self._blame = {}
		self._solver = solver

	@property 
	def priority_sequence(self):
		return self._priority_sequence
	@priority_sequence.setter
	def priority_sequence(self, pri_seq):
		self._priority_sequence = pri_seq

	@property
	def solution(self):
		return self._solution
	@solution.setter
	def solution(self, solution):
		self._solution = solution

	@property
	def blame(self):
		return self._blame
	@blame.setter
	def blame(self, blame):
		self._blame = blame

	def construct(self):
		self._solution = self._solver(self._priority_sequence)
	
	def analyze(self):
		self._blame = self._solver.calc_blame(self._priority_sequence)

	def prioritize(self):
		sorted_blame = sorted(self._blame.items(), key=operator.itemgetter(1), reverse=True)
		violated_count = 0
		for x in sorted_blame:
			if x[1] > 0:
				del self._priority_sequence[self._priority_sequence.index(x[0])]
				self._priority_sequence.insert(violated_count, x[0])
				violated_count += 1
		self._blame = sorted_blame
		return violated_count
		
		
	
class GreedyRushHour(object):
	def __init__(self, id2window, capacity):
		self._id2window = id2window
		self._capacity = capacity
		self._capacity_array = None
		self._solution = {}

	@property
	def capacity(self):
		return self._capacity
	@capacity.setter
	def capacity(self, capacity):
		self._capacity = capacity

	def init_capacity_array(self):
		return np.ones(len(self._id2window))* self._capacity

	def init_pri_seq(self):
		id2winlen= {}
		for key, value in self._id2window.iteritems():
			id2winlen[key] = value[1]-value[0]+1
		return [x[0] for x in sorted(id2winlen.items(), key=operator.itemgetter(1))]
        
	def __call__(self, pri_seq):
		self._capacity_array = self.init_capacity_array()
		for vid in pri_seq:
			start = self._id2window[vid][0]
			while start < len(self._capacity_array) and not self._capacity_array[start] > 0:
				start += 1
			self._solution[vid] = start
			self._capacity_array[start] -= 1
		return self._solution

	def calc_blame(self, pri_seq):
		blame = {}
		for id in pri_seq:
			#blame[id] = (float(self._solution[id] - self._id2window[id][1]))/float(self._id2window[id][1] - self._id2window[id][0] +1)
			blame[id] = self._solution[id] - self._id2window[id][1]
		return blame



def main():
	capacity = 2
	id2window = {}
	id2window['1'] = np.array([0,1])
	id2window['2'] = np.array([0,2])
	id2window['3'] = np.array([2,2])
	id2window['4'] = np.array([1,2])
	id2window['5'] = np.array([0,0])
	id2winlen= {}
	for key, value in id2window.iteritems():
		id2winlen[key] = value[1]-value[0]+1
	pri_seq = [x[0] for x in sorted(id2winlen.items(), key=operator.itemgetter(1))]
	pri_seq = ['1','2','3','4', '5']
	print pri_seq
	


	swo = SWO(GreedyRushHour(id2window, capacity))
	swo.priority_sequence = pri_seq
	
	swo.construct()
	swo.analyze()
	swo.prioritize()

	swo.construct()
	swo.analyze()
	swo.prioritize()




if __name__ == "__main__":
    main()

