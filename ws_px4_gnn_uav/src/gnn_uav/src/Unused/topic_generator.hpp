// Not used

// For initializing the pub/sub class instances
// T : Pub/sub class
// name_base : Node name base
// n : Number of instances
// 
// returns : Array of object pointers
template<typename T>
T** generate_topics(char* name_base, const int n) {
	T** obj = new T*[n];
	char* name = (char*)"";
    
	for(int x = 0; x < n; x++){
		std::sprintf(name, (char*)"%s_%d", name_base, x);
		obj[x] = new T(name, x);
	}
    
	return obj;
}
