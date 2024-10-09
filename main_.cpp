// // test_person.cpp
// #include <iostream>
// #include <fstream>
// #include "config/config.pb.h"
// #include "google/protobuf/io/zero_copy_stream_impl.h"
// #include "google/protobuf/text_format.h"
 
// using namespace adas;
// int main(){
//   config_path p;
//   p.set_camera_image_path("/home/adairtong/workspace/ADAS/resources/images");
//   p.set_camera_param_path("/home/adairtong/workspace/ADAS/resources/yaml");

 
//   // 将pb二进制信息保存到字符串, 序列化
//   std::string str;
//   p.SerializeToString(&str);
//   std::cout<<"str: ["<<str<<"]"<<std::endl;  
 
//   // 将pb文本信息写入文件
//   std::ofstream fw; 
//   fw.open("./config.txt", std::ios::out | std::ios::binary);
//   google::protobuf::io::OstreamOutputStream *output = new google::protobuf::io::OstreamOutputStream(&fw);
//   google::protobuf::TextFormat::Print(p, output);
 
//   delete output;
//   fw.close();
 
//   // 将pb文本信息保存到字符串
//   std::string str1;
//   google::protobuf::TextFormat::PrintToString(p, &str1);
//   std::cout<<"str1: ["<<str1<<"]"<<std::endl;
 
//   // 反序列化
//   config_path p1;
//   p1.ParseFromString(str);
//   std::cout<<"path1:"<<p1.camera_image_path()<<",path2:"<<p1.camera_param_path()<<std::endl;
 
//   return 0;
// }




// #include <iostream>
// #include <fstream>
// #include "config/config.pb.h"
// #include "google/protobuf/io/zero_copy_stream_impl.h"
// #include "google/protobuf/text_format.h"
// #include <google/protobuf/util/json_util.h>

// using namespace std;
// using namespace adas;
// using namespace google::protobuf::util;
// // 保存消息到 JSON 文件
// bool SaveMessageToJsonFile(const config_path& message, const std::string& filename) {
//     std::string json_string;
//     JsonPrintOptions options;
//     options.add_whitespace = true;
//     auto status = MessageToJsonString(message, &json_string, options);
//     if (!status.ok()) {
//         std::cerr << "Failed to serialize message to JSON: " << status.ToString() << std::endl;
//         return false;
//     }

//     std::ofstream output(filename);
//     if (!output.is_open()) {
//         std::cerr << "Failed to open file: " << filename << std::endl;
//         return false;
//     }
//     output << json_string;
//     output.close();
//     return true;
// }

// // 从 JSON 文件读取消息
// bool LoadMessageFromJsonFile(config_path& message, const std::string& filename) {
//     std::ifstream input(filename);
//     if (!input.is_open()) {
//         std::cerr << "Failed to open file: " << filename << std::endl;
//         return false;
//     }

//     std::string json_string((std::istreambuf_iterator<char>(input)),
//                             std::istreambuf_iterator<char>());
//     input.close();

//     JsonParseOptions options;
//     auto status = JsonStringToMessage(json_string, &message, options);
//     if (!status.ok()) {
//         std::cerr << "Failed to parse JSON to message: " << status.ToString() << std::endl;
//         return false;
//     }

//     return true;
// }

// int main() {
//     // 创建一个消息并设置字段
//     config_path msg;
//   msg.set_camera_image_path("/home/adairtong/workspace/ADAS/resources/images");
//   msg.set_camera_param_path("/home/adairtong/workspace/ADAS/resources/yaml");

//   single 


//     // 保存消息到 JSON 文件
//     std::string filename = "message.json";
//     // if (!SaveMessageToJsonFile(msg, filename)) {
//     //     std::cerr << "Failed to save message to JSON file." << std::endl;
//     //     return -1;
//     // }

//     // 从 JSON 文件中读取消息
//     config_path loaded_msg;
//     if (!LoadMessageFromJsonFile(loaded_msg, filename)) {
//         std::cerr << "Failed to load message from JSON file." << std::endl;
//         return -1;
//     }

//     // 打印读取的消息内容
//     std::cout << "ID: " << loaded_msg.camera_image_path() << std::endl;
//     std::cout << "Name: " << loaded_msg.camera_image_path() << std::endl;

//     return 0;
// }
#include <iostream>
#include <fstream>
#include <google/protobuf/util/json_util.h>
#include "example.pb.h"

using namespace std;
using namespace google::protobuf;
using namespace google::protobuf::util;

// 将消息转换为 JSON 字符串
string MessageToJson(const Message &message) {
    string json_string;
    JsonPrintOptions options;
    Status status = MessageToJsonString(message, &json_string, options);
    if (!status.ok()) {
        cerr << "Failed to convert message to JSON: " << status.ToString() << endl;
        return "";
    }
    return json_string;
}

// 从 JSON 字符串中解析消息
bool JsonToMessage(const string &json_string, Message &message) {
    JsonParseOptions options;
    Status status = JsonStringToMessage(json_string, &message, options);
    if (!status.ok()) {
        cerr << "Failed to parse JSON to message: " << status.ToString() << endl;
        return false;
    }
    return true;
}

// 序列化多个消息到同一个 JSON 文件
void SerializeToJsonFile(const string &filename, const Person &person, const Address &address, const Company &company) {
    // Convert messages to JSON strings
    string person_json = MessageToJson(person);
    string address_json = MessageToJson(address);
    string company_json = MessageToJson(company);

    // Create a JSON formatted string
    string json_output = "{\n";
    json_output += "  \"person\": " + person_json + ",\n";
    json_output += "  \"address\": " + address_json + ",\n";
    json_output += "  \"company\": " + company_json + "\n";
    json_output += "}";

    // Write the JSON string to file
    ofstream file(filename);
    if (!file) {
        cerr << "Failed to open file for writing: " << filename << endl;
        return;
    }
    file << json_output;
    file.close();
}

// 从 JSON 文件中解析消息
void ParseFromJsonFile(const string &filename, Person &person, Address &address, Company &company) {
    // Read JSON file
    ifstream file(filename);
    if (!file) {
        cerr << "Failed to open file for reading: " << filename << endl;
        return;
    }
    string json_string((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
    file.close();

    // Print the JSON string for debugging
    cout << "Read JSON string:\n" << json_string << endl;

    // Parse JSON string
    size_t person_start = json_string.find("\"person\":") + 10;
    size_t person_end = json_string.find("}", person_start);
    if (person_end == string::npos) person_end = json_string.find(",", person_start); // Handle cases where JSON does not end with a '}'
    string person_json = json_string.substr(person_start, person_end - person_start + 1);

    size_t address_start = json_string.find("\"address\":") + 11;
    size_t address_end = json_string.find("}", address_start);
    if (address_end == string::npos) address_end = json_string.find(",", address_start); // Handle cases where JSON does not end with a '}'
    string address_json = json_string.substr(address_start, address_end - address_start + 1);

    size_t company_start = json_string.find("\"company\":") + 11;
    size_t company_end = json_string.find("}", company_start);
    if (company_end == string::npos) company_end = json_string.find("}", company_start); // Handle cases where JSON does not end with a '}'
    string company_json = json_string.substr(company_start, company_end - company_start + 1);

    // Print extracted JSON for debugging
    cout << "Extracted Person JSON:\n" << person_json << endl;
    cout << "Extracted Address JSON:\n" << address_json << endl;
    cout << "Extracted Company JSON:\n" << company_json << endl;

    // Convert JSON strings to messages
    JsonToMessage(person_json, person);
    JsonToMessage(address_json, address);
    JsonToMessage(company_json, company);
}

int main() {
    // Create and initialize messages
    Person person;
    person.set_name("Alice");
    person.set_id(123);
    person.set_email("alice@example.com");

    Address address;
    address.set_street("123 Main St");
    address.set_city("Springfield");
    address.set_state("IL");
    address.set_zip("62704");

    Company company;
    company.set_name("Example Corp");
    company.set_location("Springfield");

    // Serialize messages to JSON file
    SerializeToJsonFile("data.json", person, address, company);

    // Deserialize messages from JSON file
    Person person2;
    Address address2;
    Company company2;

    ParseFromJsonFile("data.json", person2, address2, company2);

    // Print deserialized messages
    cout << "Deserialized Person:" << endl;
    cout << person2.DebugString() << endl;

    cout << "Deserialized Address:" << endl;
    cout << address2.DebugString() << endl;

    cout << "Deserialized Company:" << endl;
    cout << company2.name() << " " << company2.location() << endl;

    // Clean up
    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}
