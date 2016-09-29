#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector> 
#include <deque>
#include <string.h> 

using namespace std;
int startX, startY, goalX, goalY;// position of start and goal point
int numRow, numCol; 
struct node{
    int x;
    int y;
    vector<char> action;
    int cost; 
    bool visited;
    bool queued;
    char type; 
    int heuristic;
    int f;
    int health;
    int pathCost;
};
struct checkNode{
    bool queued;
    bool visited;
};
vector<vector<vector<checkNode> > > healthMap;

void makeHealthMap(){
    healthMap.resize(numRow);
    for (int i = 0; i < numRow; ++i){
        healthMap[i].resize(numCol);
        for (int j = 0; j < numCol; ++j){
            healthMap[i][j].resize(5);
            for (int k = 0; k < 5;++k){
                healthMap[i][j][k].queued = false;
                healthMap[i][j][k].visited = false;
            }
        }
    }
}


vector<vector<node> > readMapFile(vector<vector<node> > map, char *filename){
    FILE *fp = fopen(filename, "r");
    cout << filename <<endl;
    fscanf(fp, "%d %d\n", &numRow, &numCol);
    for(int i=0; i<numRow; ++i){
        char tmp_str[100];
        fgets(tmp_str,100,fp);
        printf("%s", tmp_str);
        vector<node> tmp;
        tmp.clear();
        for(int j=0; j<numCol; ++j){
            node n;
            n.x = i;
            n.y = j;
            n.visited = false;
            n.queued = false;
            n.type = tmp_str[j];
            if (n.type == ' ' || n.type =='S'){
                n.cost = 1;
            }
            else if (n.type == '*'){
                n.cost = 3;
            }
            else{
                n.cost = 0;
            }
            n.action.clear();
            n.action.push_back('*');
            n.health = 0;
            n.pathCost = 0;
            tmp.push_back(n);
        }
        map.push_back(tmp);
    }
    fclose(fp);
    return map;
}
void writeResult(vector<node> explored, char* filename){
    if (explored.empty()){
        cout << "The path planning is failed!"<<endl;
        return;
    }
    cout << "start to write the result..."<<endl;

    node tmpNode;
    //deque<char> action;
    int cost = 0;
    int numExplored;
    int pathLength = 0;
    numExplored = explored.size();
    tmpNode = explored[numExplored-1];
    cost = tmpNode.pathCost;
    pathLength = tmpNode.action.size();

    FILE *fp = fopen(strcat(filename,".txt"), "w");
    fprintf(fp,"%s","Path:\n");
    for (int i = 0; i< tmpNode.action.size(); ++i){
        fprintf(fp,"%c", tmpNode.action[i]);
    }
    fprintf(fp, "\n%s %s %d", "Path","length:", pathLength);
    fprintf(fp, "\n%s %s %d", "Path", "cost:", cost);
    fprintf(fp, "\n%s %s %s %d", "#","Nodes", "Examined:", numExplored);
    fclose(fp);
    cout << "The result is written."<<endl;

}

void findStartAndGoal(vector<vector<node> > map){
    int i,j;
    for (i = 0; i < numRow; ++i){
        for (j = 0; j < numCol; ++j){
            if (map[i][j].type == 'S'){
                startX = i;
                startY = j;
            }
            else if (map[i][j].type == 'G'){
                goalX = i;
                goalY = j;
            }
        }
    }
}

vector<vector<node> > zeroHeuristics(vector<vector<node> > map){
    for (int i = 0; i < numRow; ++i){
        for (int j = 0; j < numCol; ++j){
            map[i][j].heuristic = 0;
        }
    }
    map[startX][startY].health = 5;
    return map;
}

vector<vector<node> > findHeuristics(vector<vector<node> > map){
    for (int i = 0; i < numRow; ++i){
        for (int j = 0; j < numCol; ++j){
            map[i][j].heuristic = abs(i-goalX)+abs(j-goalY);
        }
    }
    map[startX][startY].health = 5;
    return map;
}
vector<vector<node> > horizonHeuristics(vector<vector<node> > map){
    for (int i = 0; i < numRow; ++i){
        for (int j = 0; j < numCol; ++j){
            map[i][j].heuristic = abs(i-goalX)/4+abs(j-goalY);
        }
    }
    map[startX][startY].health = 5;
    return map;
}

vector<vector<node> > wallHeuristics(vector<vector<node> > map){

    for (int j = 0; j < numCol; ++j){
        for (int i = 0; i < numRow; ++i){
            int pos = i;
            int move = 0;
            int min = numRow + numCol;
            if (map[i][j].type == 'X' || map[i][j].type == 'O'){
                map[i][j].heuristic = min;
                continue;

            }
            if (j == numCol-2){
                min = abs(goalX-i);
            }
            else{
                while(pos-move > 0){
                    if (map[pos-move][j+1].type == ' ' || map[pos-move][j+1].type == 'G'){
                        min = move +abs(j-goalY)+abs(pos-move-goalX);
                        break;
                    }
                    else if (map[pos-move][j].type == 'X' ||map[pos-move][j].type == 'O'){
                        break;
                    }
                    ++move;
                }
                move = 0;
                while(pos+move < numRow){
                    if (map[pos+move][j+1].type == ' '){
                        if ((move + abs(j-goalY)+abs(pos+move-goalX))<= min){
                            min = move +abs(j-goalY)+abs(pos+move-goalX);
                        }
                        break;
                    }
                    else if (map[pos+move][j].type == 'X' ||map[pos+move][j].type == 'O'){
                        break;
                    }
                    ++move;
                }
            }
            map[i][j].heuristic = min;
        }
    }
    map[startX][startY].health = 5;
    return map;
}

vector<char> findAction(vector<vector<node> > map, node Node, vector<char> action, char methodFlag){
    int x = Node.x;
    int y = Node.y;
    action.clear();
    if (methodFlag == 'D'){
        if (y<(numCol-1) && map[x+1][y].type != 'X' && map[x+1][y].type != 'O'){
            action.push_back('S');
        }
        if (x<(numRow-1) && map[x][y+1].type != 'X' && map[x][y+1].type != 'O'){
            action.push_back('E');
        }
        if (y>0 && map[x][y-1].type != 'X' && map[x][y-1].type != 'O'){
            action.push_back('W');
        }
        if (x>0 && map[x-1][y].type != 'X' && map[x-1][y].type != 'O'){
            action.push_back('N');
        }
    }
    else{
        if (x>0 && map[x-1][y].type != 'X' && map[x-1][y].type != 'O'){
            action.push_back('N');
        }
        if (y>0 && map[x][y-1].type != 'X' && map[x][y-1].type != 'O'){
            action.push_back('W');
        }
        if (x<(numRow-1) && map[x][y+1].type != 'X' && map[x][y+1].type != 'O'){
            action.push_back('E');
        }
        if (y<(numCol-1) && map[x+1][y].type != 'X' && map[x+1][y].type != 'O'){
            action.push_back('S');
        }
    }
    return action;
}

node findChild(vector<vector<node> > map, node Node, char action, node child){
    int x,y;
    switch (action){
        case 'N':
            x = Node.x - 1;
            y = Node.y;
            child = map[x][y];
            break;
        case 'W':
            x = Node.x;
            y = Node.y - 1;
            child = map[x][y];
            break;
        case 'E':
            x = Node.x;
            y = Node.y + 1;
            child = map[x][y];
            break;
        case 'S':
            x = Node.x + 1;
            y = Node.y;
            child = map[x][y];
            break;
    }
    return child;
}
node healthCheck(vector<vector<node> > map, node child, node Node){
    int x = child.x;
    int y = child.y;
    char neighbours[4] = {map[x-1][y].type, map[x][y-1].type, map[x][y+1].type, map[x+1][y].type};

    child.health = Node.health;
    for (int i = 0; i < 4; ++i){
        if (neighbours[i] == 'O'){
            if (child.type == '*'){
                child.health = Node.health - 3;
            }
            else{
                child.health = Node.health - 1;
            }
        }
    }
    return child;
}
deque<node> insertChild(node child, deque<node> frontier){
    int i = 0;
    if (frontier.empty()){
        frontier.push_back(child);
        return frontier;
    }
    else if (child.f > frontier[frontier.size()-1].f){
        frontier.push_back(child);
        return frontier;
    }

    else{
        for (deque<node>::iterator j = frontier.begin();j < frontier.end();++j,++i){
            if(child.f < frontier[i].f){
                frontier.insert(j,child);
                return frontier;
            }
            // reorder based on NWES and health
            if (child.f == frontier[i].f){
                if (child.x < frontier[i].x){
                    frontier.insert(j,child);
                    return frontier;
                }
                else if (child.x == frontier[i].x){
                    if (child.y < frontier[i].y){
                        frontier.insert(j,child);
                        return frontier;
                    }
                    else if (child.y == frontier[i].y){
                        if (child.health > frontier[i].health){
                            frontier.insert(j,child);
                            return frontier;
                        }
                    }
                }
                if (child.f == frontier[frontier.size()-1].f){
                    if(child.x >= frontier[frontier.size()-1].x ){
                        frontier.push_back(child);
                        return frontier;
                    }
                    
                    else if (child.x == frontier[frontier.size()-1].x){
                        if (child.y < frontier[frontier.size()-1].y){
                            frontier.insert(j,child);
                            return frontier;
                        }
                        else if (child.y == frontier[frontier.size()-1].y){
                            if (child.health > frontier[frontier.size()-1].health){
                                frontier.insert(j,child);
                                return frontier;
                            }
                        }
                    }
                }
            }
            //i++i;
        }
    }
    return frontier;
}

vector<node> aStar(vector<vector<node> >map, deque<node> frontier, vector<node> explored, char methodFlag){

    node Node;
    node child;
    vector<char> action;

    frontier.push_back(map[startX][startY]);
    if (frontier[0].type == 'G'){
        return explored;
    }
    while(!frontier.empty()){
        Node = frontier.front();
        
        frontier.pop_front();
        healthMap[Node.x][Node.y][Node.health-1].queued = false;
        map[Node.x][Node.y].queued = false;

        explored.push_back(Node);
        healthMap[Node.x][Node.y][Node.health-1].visited = true;
        if (Node.type == 'G'){
            explored[explored.size()-1].action.erase(explored[explored.size()-1].action.begin());
            return explored;
        }
        action = findAction(map, Node, action, methodFlag);
        for (int i = 0; i < action.size(); ++i){
            node child;
            child = findChild(map, Node, action[i], child);
            child = healthCheck(map, child, Node);
            if (child.health <= 0){
                continue;
            }
            child.pathCost = Node.pathCost + Node.cost;
            child.f = child.heuristic + child.pathCost;
            child.action.clear();
            
            for (int k = 0; k < Node.action.size(); ++k ){// save the path 
                child.action.push_back(Node.action[k]);

            }
            child.action.push_back(action[i]);
            if (!healthMap[child.x][child.y][child.health-1].queued && !healthMap[child.x][child.y][child.health-1].visited){
                frontier = insertChild(child, frontier);
                healthMap[child.x][child.y][child.health-1].queued = true;
            }
            else if(healthMap[child.x][child.y][child.health-1].queued && !healthMap[child.x][child.y][child.health-1].visited){// if health equal,replace if the cost is smaller
                for (int j = 0; j < frontier.size(); ++j){
                    if (child.x == frontier[j].x && child.y == frontier[j].y){
                        if (child.health == frontier[j].health && child.f < frontier[j].f){

                            frontier.erase(frontier.begin()+j);
                            frontier = insertChild(child, frontier);
                            //healthMap[frontier[j].x][frontier[j].y][frontier[j].health-1].queued = false;
                            //healthMap[child.x][child.y][child.health-1].queued = true;
                        }
                    }
                }
            }
        }
    }
    return explored;
}

vector<node> GBS(vector<vector<node> >map, deque<node> frontier, vector<node> explored, char methodFlag){

    node Node;
    node child;
    vector<char> action;

    frontier.push_back(map[startX][startY]);
    if (frontier[0].type == 'G'){
        return explored;
    }
    while(!frontier.empty()){
        Node = frontier.front();
        
        frontier.pop_front();
        healthMap[Node.x][Node.y][Node.health-1].queued = false;
        map[Node.x][Node.y].queued = false;

        explored.push_back(Node);
        healthMap[Node.x][Node.y][Node.health-1].visited = true;
        if (Node.type == 'G'){
            explored[explored.size()-1].action.erase(explored[explored.size()-1].action.begin());
            return explored;
        }
        action = findAction(map, Node, action, methodFlag);
        for (int i = 0; i < action.size(); ++i){
            node child;
            child = findChild(map, Node, action[i], child);
            child = healthCheck(map, child, Node);
            if (child.health <= 0){
                continue;
            }
            child.pathCost = Node.pathCost + Node.cost;
            child.f = child.heuristic;
            child.action.clear();
            
            for (int k = 0; k < Node.action.size(); ++k ){// save the path 
                child.action.push_back(Node.action[k]);

            }
            child.action.push_back(action[i]);
            if (!healthMap[child.x][child.y][child.health-1].queued && !healthMap[child.x][child.y][child.health-1].visited){
                frontier = insertChild(child, frontier);
                healthMap[child.x][child.y][child.health-1].queued = true;
            }
            else if(healthMap[child.x][child.y][child.health-1].queued && !healthMap[child.x][child.y][child.health-1].visited){// if health equal,replace if the cost is smaller
                for (int j = 0; j < frontier.size(); ++j){
                    if (child.x == frontier[j].x && child.y == frontier[j].y){
                        if (child.health == frontier[j].health && child.f < frontier[j].f){

                            frontier.erase(frontier.begin()+j);
                            frontier = insertChild(child, frontier);
                            //healthMap[frontier[j].x][frontier[j].y][frontier[j].health-1].queued = false;
                            //healthMap[child.x][child.y][child.health-1].queued = true;
                        }
                    }
                }
            }
        }
    }
    return explored;
}

vector<node> UCS(vector<vector<node> >map, deque<node> frontier, vector<node> explored, char methodFlag){

    node Node;
    node child;
    vector<char> action;

    frontier.push_back(map[startX][startY]);
    if (frontier[0].type == 'G'){
        return explored;
    }
    while(!frontier.empty()){
        Node = frontier.front();
        
        frontier.pop_front();
        map[Node.x][Node.y].queued = false;
        
        explored.push_back(Node);
        map[Node.x][Node.y].visited = true;
        if (Node.type == 'G'){
            explored[explored.size()-1].action.erase(explored[explored.size()-1].action.begin());
            return explored;
        }
        action = findAction(map, Node, action, methodFlag);
        for (int i = 0; i < action.size(); ++i){
            node child;
            child = findChild(map, Node, action[i], child);
            child.pathCost = Node.pathCost + Node.cost;
            child.f = child.pathCost;
            child.action.clear();
            
            for (int k = 0; k < Node.action.size(); ++k ){// save the path 
                child.action.push_back(Node.action[k]);

            }
            child.action.push_back(action[i]);
            if (!map[child.x][child.y].queued && !map[child.x][child.y].visited){
                frontier = insertChild(child, frontier);
                map[child.x][child.y].queued = true;
            }
            else if(map[child.x][child.y].queued && !map[child.x][child.y].visited){// if health equal,replace if the cost is smaller
                for (int j = 0; j < frontier.size(); ++j){
                    if (child.x == frontier[j].x && child.y == frontier[j].y){
                        if (child.f < frontier[j].f){

                            frontier.erase(frontier.begin()+j);
                            frontier = insertChild(child, frontier);
                            //healthMap[frontier[j].x][frontier[j].y][frontier[j].health-1].queued = false;
                            //healthMap[child.x][child.y][child.health-1].queued = true;
                        }
                    }
                }
            }
        }
    }
    return explored;
}
vector<node> BFS(vector<vector<node> > map, deque<node> frontier, vector<node> explored, char methodFlag){
    node Node;
    node child;

    vector<char> action;
    frontier.push_back(map[startX][startY]);
    if (frontier[0].type == 'G'){
        return explored;
    }
    while(!frontier.empty()){
        Node = frontier.front();
        frontier.pop_front();
        map[Node.x][Node.y].queued = false;
        
        explored.push_back(Node);
        map[Node.x][Node.y].visited = true;
        action = findAction(map, Node, action, methodFlag);

        for (int i = 0; i<action.size();++i){
            node child;
            child = findChild(map, Node, action[i], child);
            child.pathCost = Node.pathCost + Node.cost;
            child.action.clear();
            for (int k = 0; k < Node.action.size(); ++k ){// save the path 
                child.action.push_back(Node.action[k]);

            }
            child.action.push_back(action[i]);
            if (!child.visited && !child.queued){
                if (child.type == 'G'){
                    explored.push_back(child);
                    explored[explored.size()-1].action.erase(explored[explored.size()-1].action.begin());
                    return explored;
                }
                frontier.push_back(child);
                map[child.x][child.y].queued = true;
            }
        }
    }

    return explored;
}

vector<node> DFS(vector<vector<node> > map, deque<node> frontier, vector<node> explored, char methodFlag){
    node Node;
    node child;

    vector<char> action;
    frontier.push_back(map[startX][startY]);
    if (frontier[0].type == 'G'){
        return explored;
    }
    while(!frontier.empty()){
        Node = frontier.back();
        frontier.pop_back();
        map[Node.x][Node.y].queued = false;
        
        explored.push_back(Node);
        map[Node.x][Node.y].visited = true;
        action = findAction(map, Node, action, methodFlag);

        for (int i = 0; i<action.size();++i){
            node child;
            child = findChild(map, Node, action[i], child);
            child.pathCost = Node.pathCost + Node.cost;
            child.action.clear();
            for (int k = 0; k < Node.action.size(); ++k ){// save the path 
                child.action.push_back(Node.action[k]);

            }
            child.action.push_back(action[i]);
            if (!child.visited && !child.queued){
                if (child.type == 'G'){
                    explored.push_back(child);
                    explored[explored.size()-1].action.erase(explored[explored.size()-1].action.begin());
                    return explored;
                }
                frontier.push_back(child);
                map[child.x][child.y].queued = true;
            }
        }
    }

    return explored;
}

int main(int argc,char* argv[]){
    bool flag;
    vector<vector<node> > map;
    deque<node> frontier;
    vector<node> explored;
    char methodFlag = 'B'; // DFS and BFS are using different action sorting method 

    cout << "start to read the map..."<<endl;
    map = readMapFile(map,argv[2]);
    findStartAndGoal(map);
    map = findHeuristics(map);
    makeHealthMap();

    if (!strcmp(argv[1],"DFS")){
        cout << "Doing prob 1" << endl;
        methodFlag = 'D';
        explored = DFS(map, frontier, explored, methodFlag);
    }
    else if (!strcmp(argv[1], "BFS")){
        cout << "Doing prob 2" << endl;
        explored = BFS(map, frontier, explored, methodFlag);
    }
    else if (!strcmp(argv[1], "UCS")){
        cout << "Doing prob 3" << endl;
        explored = UCS(map, frontier, explored, methodFlag);
    }
    else if (!strcmp(argv[1], "GBS")){
        cout << "Doing prob 4" << endl;
        explored = GBS(map, frontier, explored, methodFlag);
    }
    else if (!strcmp(argv[1], "ASTAR")){
        cout << "Doing prob 5" << endl;
        explored = aStar(map, frontier, explored, methodFlag);
    }
    else if (!strcmp(argv[1], "Prob6_1")){
        cout << "Doing prob 6" << endl;
        map = zeroHeuristics(map);
        explored = aStar(map, frontier, explored, methodFlag);
    }
    else if (!strcmp(argv[1], "Prob6_2")){
        map = findHeuristics(map);
        explored = aStar(map, frontier, explored, methodFlag);
    }
    else if (!strcmp(argv[1], "Prob6_3")){
        map = horizonHeuristics(map);
        explored = aStar(map, frontier, explored, methodFlag);
    }
    else if (!strcmp(argv[1], "Prob6_4")){
        map = wallHeuristics(map);
        explored = aStar(map, frontier, explored, methodFlag);
    }

    // for (int i = 0; i < numRow; ++i){
    //     for (int j =0;  j < numCol; ++j){
    //         cout<<map[i][j].heuristic<<",";
    //     }
    //     cout<<endl;
    // }
    cout << "Path planning is done!"<<endl;

    writeResult(explored, argv[1]); 
    return 0;
}


