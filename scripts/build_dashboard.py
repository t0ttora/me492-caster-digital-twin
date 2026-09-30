#!/usr/bin/env python3
"""Compatibility entry point for the consolidated research journal builder."""
from build_site import api_all, build, check

if __name__ == '__main__':
    check()
    build(api_all('issues?state=all'), api_all('issues/comments?sort=created&direction=asc'))
