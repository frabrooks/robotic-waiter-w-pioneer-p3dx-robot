#! /usr/bin/env python

from enum import Enum

Status = Enum('Status',
    'queued to_bar to_location conformation_wait completed deleted')


class Job:

    def __init__(self, goal, item, jobid):
        self.goal = goal
        self.item = item
        self.jobid = jobid
        self.status = Status.queued

    def delete(self):
        self.status = Status.deleted

    def set_status(self, status):
        self.status = status

    def __str__(self):
        return(self.goal + '|' +
               self.item + '|' +
               self.jobid + '|' +
               self.status.name)
