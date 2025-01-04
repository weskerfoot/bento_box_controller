#! /usr/bin/env bash
#curl -XPOST http://192.168.0.41/sensor -d '{"voc_max_threshold": 150, "voc_min_threshold" : 140}'
curl -XPOST http://192.168.0.41/sensor -d '{"voc_max_threshold": 130, "voc_min_threshold" : 120}'
