import time
from datetime import datetime as dt
"""
Building a website blocker by Supratim Das
"""
host_temp = "hosts"
hosts_path = "/etc/hosts"
redirect = "127.0.0.1"
website_list = ["www.facebook.com", "facebook.com", "www.xvideos.net"]

# Now I want this script to run continuously, so I want a while loop
while True:
    #time.sleep(5)
    if(dt(dt.now().year,dt.now().month,dt.now().day,8) < dt.now() < dt(dt.now().year,dt.now().month,dt.now().day,17)):
        print("Working hours!!")
        with open(hosts_path,'r+') as file:
            content = file.read()
            #print(content)
            for website in website_list:
                if website in content:
                    pass
                else:
                    file.write(redirect + "\t" + website + "\n")

    else:
        print("Fun hours!!")
        with open(hosts_path,'r+') as file:
            content = file.readlines()
            # 0 means to the start of the file, means overwrite
            # blank means to the end of the file means append
            file.seek(0)
            for line in content:
                #print(line)
                if not any (website in line for website in website_list):
                    file.write(line)
            file.truncate()
    time.sleep(5)
