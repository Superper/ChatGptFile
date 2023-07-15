#ifndef FTPUP_H
#define FTPUP_H
#include <curl/curl.h>
#include <iostream>
#include <string>

class FTPUploader
{
  public:
    FTPUploader()
    {
        curl_global_init(CURL_GLOBAL_DEFAULT);
        curl = curl_easy_init();
    }

    ~FTPUploader()
    {
        curl_easy_cleanup(curl);
        curl_global_cleanup();
    }

    bool uploadData(const void *data, size_t dataSize, const std::string &url, const std::string &userpwd)
    {
        if (!curl)
        {
            std::cerr << "Failed to initialize libcurl" << std::endl;
            return false;
        }

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_USERPWD, userpwd.c_str());
        curl_easy_setopt(curl, CURLOPT_READFUNCTION, ReadDataCallback);
        curl_easy_setopt(curl, CURLOPT_READDATA, data);
        curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
        curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE, static_cast<curl_off_t>(dataSize));

        CURLcode res = curl_easy_perform(curl);
        if (res != CURLE_OK)
        {
            std::cerr << "Failed to upload data: " << curl_easy_strerror(res) << std::endl;
            return false;
        }

        return true;
    }

  private:
    static size_t ReadDataCallback(void *buffer, size_t size, size_t nmemb, void *userp)
    {
        const char *data = static_cast<const char *>(userp);
        size_t dataSize = size * nmemb;

        if (dataSize > 0)
        {
            memcpy(buffer, data, dataSize);
            return dataSize;
        }

        return 0;
    }

    CURL *curl;
};
#endif