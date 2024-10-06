#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 20 17:50:44 2019

@author: kallil
"""

import xml.dom.minidom

def parseSup(filename):
    
    doc = xml.dom.minidom.parse(filename)
    # Só irá fazer o parsing caso o tipo do autômato seja um supervisor. Caso contrário, 
    # não fará o parsing de nada
    doc_verify = doc.getElementsByTagName("Automaton")
    for verify in doc_verify:
        
        if(verify.getAttribute("type") == "Supervisor"):
            doc = verify
            break
    
    doc_actions = verify.getElementsByTagName("Event")
    doc_states = verify.getElementsByTagName("State")
    doc_transitions = verify.getElementsByTagName("Transition")
    actions = []
    controllable = []
    ncontrollable = []
    states = []
    terminal = []
    transitions = []
    
    for action in doc_actions:
        actions.append([int(action.getAttribute("id")), action.getAttribute("label")])
        if(not(action.getAttribute("controllable"))):
            controllable.append(int(action.getAttribute("id")))
        else:
            ncontrollable.append(int(action.getAttribute("id")))
    for state in doc_states:
        states.append(int(state.getAttribute("id")))
        if(state.getAttribute("accepting")):
            terminal.append(int(state.getAttribute("id")))
        if(state.getAttribute("initial")):
           initial_state = int(state.getAttribute("id")) 
           
    for transition in doc_transitions:
        transitions.append([int(transition.getAttribute("source")),
                            int(transition.getAttribute("dest")), 
                            int(transition.getAttribute("event"))])
        
    return actions,controllable, ncontrollable, states, terminal, initial_state, transitions
        

def parse(filename):
    
    doc = xml.dom.minidom.parse(filename)
    doc_verify = doc.getElementsByTagName("Automaton")
    verify = doc_verify[0]      
    doc_actions = verify.getElementsByTagName("Event")
    doc_states = verify.getElementsByTagName("State")
    doc_transitions = verify.getElementsByTagName("Transition")
    actions = []
    controllable = []
    ncontrollable = []
    states = []
    terminal = []
    transitions = []
    
    for action in doc_actions:
        actions.append([int(action.getAttribute("id")), action.getAttribute("label")])
        if(not(action.getAttribute("controllable"))):
            controllable.append(int(action.getAttribute("id")))
        else:
            ncontrollable.append(int(action.getAttribute("id")))
    for state in doc_states:
        states.append(int(state.getAttribute("id")))
        if(state.getAttribute("accepting")):
            terminal.append(int(state.getAttribute("id")))
        if(state.getAttribute("initial")):
           initial_state = int(state.getAttribute("id")) 
           
    for transition in doc_transitions:
        transitions.append([int(transition.getAttribute("source")),
                            int(transition.getAttribute("dest")), 
                            int(transition.getAttribute("event"))])
        
    return actions,controllable, ncontrollable, states, terminal, initial_state, transitions

