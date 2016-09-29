To run the code, please just type "./pathPlanning methodName testMapName" in command line.

Acceptable methodName:
					DFS     : for depth first search
					BSF     : for breadth first search
					UCS     : for uniformed cost search
					GBS     : for greedy best search
					ASTAR   : for a* search
					Prob6_1 : for a* search with zero heuristic
					Prob6_2 : for a* search with manhattan heuristic
					Prob6_3 : for a* search with horizontally move heuristic
					Prob6_4 : for a* search with wall heuristic

Acceptable testMapName:
					test_map1.txt
					test_map2.txt
					test_map3.txt
					large_map1.txt
					large_map2.txt

Example of running the code :

			./pathPlanning DFS test_map1.txt (run depth first search on test_map1.txt) 

After running the code, the result would be written to a file named "methodName.txt".

All answers are listed in "answers.txt".