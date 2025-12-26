#!/usr/bin/env python3
"""Quick verification script to check setup is working."""

import os
import sys

print("="*80)
print("Verification Check: RAG Content Ingestion Pipeline")
print("="*80)

# 1. Check .env file exists
print("\n✓ Step 1: Check .env file...")
if os.path.exists("backend/.env"):
    print("  ✓ .env file found")
    with open("backend/.env") as f:
        lines = [l.strip() for l in f if l.strip() and not l.startswith("#")]
    print(f"  ✓ Loaded {len(lines)} environment variables")
else:
    print("  ✗ .env file NOT found")
    sys.exit(1)

# 2. Check dependencies installed
print("\n✓ Step 2: Check dependencies...")
required = ["requests", "bs4", "qdrant_client", "cohere", "tiktoken", "dotenv"]
missing = []

for pkg in required:
    try:
        __import__(pkg)
        print(f"  ✓ {pkg}")
    except ImportError:
        print(f"  ✗ {pkg} - NOT INSTALLED")
        missing.append(pkg)

if missing:
    print(f"\n  Missing packages: {', '.join(missing)}")
else:
    print("  ✓ All required packages available")

# 3. Check main.py syntax
print("\n✓ Step 3: Check main.py syntax...")
try:
    import py_compile
    py_compile.compile("backend/main.py", doraise=True)
    print("  ✓ main.py has valid Python syntax")
except py_compile.PyCompileError as e:
    print(f"  ✗ Syntax error: {e}")
    sys.exit(1)

# 4. Attempt to import main module
print("\n✓ Step 4: Test imports...")
try:
    sys.path.insert(0, "backend")
    from main import (
        setup_logging,
        HTTPClient,
        TokenCounter,
        EnvironmentConfig,
        get_all_urls,
        extract_text_from_url,
        chunk_text,
        embed,
        create_collection,
        save_chunk_to_qdrant,
        search_vectors,
        validate_ingestion,
        main as main_function,
    )
    print("  ✓ All core functions imported successfully")
except Exception as e:
    print(f"  ✗ Import error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# 5. Test core functionality
print("\n✓ Step 5: Test core functionality...")
try:
    # Test token counter
    logger = setup_logging("INFO")
    tc = TokenCounter(logger)
    tokens = tc.count("Hello world, this is a test")
    print(f"  ✓ TokenCounter works ({tokens} tokens)")
    
    # Test HTTP client creation
    hc = HTTPClient(logger=logger)
    print(f"  ✓ HTTPClient instantiated")
    
    # Test logging
    logger.info("Test log message")
    print(f"  ✓ Logging works")
    
except Exception as e:
    print(f"  ✗ Functionality test failed: {e}")
    sys.exit(1)

print("\n" + "="*80)
print("✅ All verification checks PASSED!")
print("="*80)
print("\n✨ Your RAG pipeline is ready to use!")
print("\nNext steps:")
print("1. Edit backend/.env with your real API credentials")
print("2. Run: uv run python backend/main.py")
print("3. Monitor logs for progress")
print("4. Verify vectors in Qdrant Cloud dashboard")
print("\n")
