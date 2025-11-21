from setuptools import setup, find_packages

setup(
    name='r2d2_llm',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'python-dotenv',
        'requests',
    ],
    entry_points={
        'console_scripts': [
            'tts_node = r2d2_llm.tts_node:main',
            'stt_node = r2d2_llm.stt_node:main',
            'grok_fallback = r2d2_llm.grok_fallback:query_grok',  # For easy testing
        ],
    },
)
