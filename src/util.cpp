#include <util.h>

void errorCout(const std::string& str, const std::string& file, const int& line){
    std::cout << "\n###ERROR:\n" << str << "###ERROR IN FILE: " << file << ", IN LINE: " << line << "\n" << std::endl;
}

std::string getTime(){
    std::string out;
    time_t timer;
    struct tm* timeinfo;
    char buff[80];
    
    time(&timer);
    timeinfo = localtime(&timer);
    strftime(buff, 80, "%F-%H-%M-%S", timeinfo);
    out = buff;
    return out;
}



