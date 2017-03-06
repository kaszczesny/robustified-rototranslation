#!/usr/bin/env bash

ARTICLE=Szczesny_Twardowski_MSc

./clean.sh

pdflatex -halt-on-error -interaction nonstopmode ${ARTICLE} &&
bibtex ${ARTICLE} &&
pdflatex -halt-on-error -interaction nonstopmode ${ARTICLE} &&
pdflatex -halt-on-error -interaction nonstopmode ${ARTICLE}

exit $?
