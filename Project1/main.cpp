#include<iostream>
#include <fstream>
#include <ostream>


int main() {
    std::string target_click_path, target_mms_path;

	std::ofstream target_click, target_mms;
    target_click.open(target_click_path);
    target_mms.open(target_mms_path);

	return 0;
}