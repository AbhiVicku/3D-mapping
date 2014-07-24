#include <iostream>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace std;
using namespace octomap;

int main(){
	cout<<endl;
	cout<<" generating map "<<endl;
	
	// generating tree with resolution 0.1 m
	OcTree tree(0.1);
	
	// insert some measurements into it 
	for(int x = -20;x<20;x++){
		for(int y = -20;y<20;y++){
			for(int z = -20;z < 20;z++){
				point3d endpoint((float) x*0.02f -1.0f,(float) y*0.02f -1.0f,(float) z*0.02f - 1.0f);
				tree.updateNode(endpoint, true);
				
			}
		}
	}
	
	//insert some measurements of free cells into the previous tree
	for(int x = -30;x<30;x++){
		for(int y = -30;y<30;y++){
			for(int z = -30;z<30;z++){
				point3d endpoint ((float) x*0.02f-1.0f,(float) y*0.02f-1.0f,(float) z*0.02f - 1.0f);
				tree.updateNode(endpoint,false);
			}
		}
	
	}
	
	cout<<endl;
	tree.writeBinary("simple_tree.bt");
	cout<<"Finished"<<endl;
	return 0;
	

}
