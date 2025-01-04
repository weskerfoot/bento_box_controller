#! /usr/bin/env bash
curl -XPOST http://192.168.0.41/sensor -d '{"voc_max_threshold": 120}'
