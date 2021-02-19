#ifndef __managedAlloc_INCLUDED__   // if x.h hasn't been included yet...
#define __managedAlloc_INCLUDED__   //   #define this so the compiler knows it has been included

#include <stdio.h>
#include <string.h>
#include <stdlib.h>  

#define PRINT_NON_ERROR 1
#define allocateMem allocateMem//(T** mem, int length, memTypes memType, const char * memName)
#define deleteAllAllocations deleteAllAllocations
enum memTypes {
	hostMem,
	hostAllignedMem,
	NONE
};

struct managedMem {
	struct managedMem * next;
	void ** mem;
	memTypes type;
	char name[64];
};

class managedMemClass {

private:
	struct managedMem *head = NULL;
	struct managedMem *curr = NULL;
	UINT totalManagedMem = 0;

	//Helpers
	std::wstring s2ws(const std::string& str)
	{
		int size_needed = MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), NULL, 0);
		std::wstring wstrTo(size_needed, 0);
		MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), &wstrTo[0], size_needed);
		return wstrTo;
	}
	void printToDebug(UINT size, float ManagedMemMB, const char * memName) {

		std::wstring text = L" Allocating: " + s2ws(memName) + L" Size = " + std::to_wstring(size) + L" (current total = " + std::to_wstring(ManagedMemMB)  + L" mb)" +
			L"\n";

		::OutputDebugString(text.c_str());
	}

	//Main Memory Code
	struct managedMem* create_list()
	{
#if PRINT_NON_ERROR
		fprintf(stderr, "Creating list with headnode as [%d]\n", NONE);
#endif
		struct managedMem *ptr = (struct managedMem*)malloc(sizeof(struct managedMem));
		if (NULL == ptr)
		{
			fprintf(stderr, "\n Node creation failed \n");
			return NULL;
		}

		ptr->next = NULL;
		ptr->mem = NULL;
		ptr->type = NONE;
		const char * memName = "headNode";
		strcpy_s(ptr->name, 64, memName);

		head = curr = ptr;
		return ptr;
	}
	struct managedMem* add_to_list(void** memptr, memTypes memTy, const char * memName)
	{
		if (NULL == head)
		{
			create_list();
		}
#if PRINT_NON_ERROR
		fprintf(stderr, "Adding node [%d] %s\n", memTy, memName);
#endif
		struct managedMem *ptr = (struct managedMem*)malloc(sizeof(struct managedMem));
		if (NULL == ptr)
		{
			fprintf(stderr, "\n Node creation failed \n");
			return NULL;
		}
		ptr->next = NULL;
		ptr->mem = memptr;
		ptr->type = memTy;
		strcpy_s(ptr->name, 64, memName);

		curr->next = ptr;
		curr = ptr;

		return ptr;
	}

	template <class T> void freeMem(T** mem, int memType, const char * memName) {
		if (memType == hostMem) {
			free(*mem);
			*mem = NULL;
		}
		else if (memType == hostAllignedMem) {
			_aligned_free(*mem);
			*mem = NULL;
		}
	}

public:
	template <class T> void allocateMem(T** mem, int length, memTypes memType, const char * memName) {
		UINT allocSize = sizeof(T)*length;
		if (memType == hostMem) {
			*mem = (T*)malloc(allocSize);
			memset(*mem, 0, (allocSize));
			if (!*mem) {
				fprintf(stderr, "Error %s\n", memName);
			}
			else {
				add_to_list((void**)mem, memType, memName);
			}
		}
		else if (memType == hostAllignedMem) {
			*mem = (T*)_aligned_malloc(allocSize, 32);
			if (!*mem) {
				fprintf(stderr, "Error %s\n", memName);
			}
			else {
				add_to_list((void**)mem, memType, memName);
			}
		}
		//Watching allocation amount
		totalManagedMem += allocSize;
		float managedMemMB = totalManagedMem / 1000000.0f;
		printToDebug(allocSize, managedMemMB, memName);
	}

	void deleteAllAllocations()
	{
		struct managedMem *ptr = head; //Get first alloc
#if PRINT_NON_ERROR
		fprintf(stderr, "\n -------Printing list Start------- \n");
#endif
		while (ptr != NULL)
		{
#if PRINT_NON_ERROR
			fprintf(stderr, "deleting [%d] %s\n", ptr->type, ptr->name);
#endif

			freeMem(ptr->mem, ptr->type, ptr->name);
			//Delete List Items
			struct managedMem *del = ptr;
			ptr = ptr->next;
			free(del); del = NULL;
		}
#if PRINT_NON_ERROR
		fprintf(stderr, "\n -------Printing list End------- \n");
#endif

		head = NULL;
		curr = NULL;

		return;
	}
};

#endif