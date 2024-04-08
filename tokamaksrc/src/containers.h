/*************************************************************************
 *                                                                       *
 * Tokamak Physics Engine, Copyright (C) 2002-2007 David Lam.            *
 * All rights reserved.  Email: david@tokamakphysics.com                 *
 *                       Web: www.tokamakphysics.com                     *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT for more details.                                         *
 *                                                                       *
 *************************************************************************/

#ifndef CONTAINERS_H
#define CONTAINERS_H

#define _CRT_SECURE_DEPRECATE_MEMORY
#include <memory.h>
#include <string.h>

#define PLACEMENT_MAGIC \
	public:\
	NEINLINE void * operator new(size_t s,void * addr)\
	{\
		(void)s;\
		return addr;\
	}\
	NEINLINE void * operator new[] (size_t s,void * addr)\
	{\
		(void)s;\
		return addr;\
	}\
	NEINLINE void operator delete(void *)\
	{\
	}\
	NEINLINE void operator delete[](void *)\
	{\
	}\
	NEINLINE void operator delete(void *, void *)\
	{\
	}\
	NEINLINE void operator delete[](void *, void *)\
	{\
	}

template <class T, int initFixedSize = 1> class neSimpleArray
{
public:
	PLACEMENT_MAGIC

	NEINLINE bool IsFixedSize()
	{
		return (initFixedSize != 1);
	}

	NEINLINE neSimpleArray()
	{
		if (IsFixedSize())
		{
			data = initArray;
			nextFree = data;
			size = initFixedSize;
			alloc = nullptr;
			growBy = 0;
			usedSize = 0;
		}
		else
		{
			data = nullptr;
			nextFree = nullptr;
			size = 0;
			alloc = &allocDef;
			growBy = 0;
			usedSize = 0;
		}
	}
	NEINLINE ~neSimpleArray()
	{
		Free();
	}
	NEINLINE bool Reserve(int32_t n, neAllocatorAbstract * al = nullptr, int32_t _growBy = 0) 
	{
		if (IsFixedSize())
		{
			ASSERT(0);
			return false;
		}
		Free();

		if (al)
			alloc = al;
		else
			alloc = & allocDef;
		
		growBy = _growBy;

		neByte * mem = alloc->Alloc(sizeof(T) * n);

		data = (T*) mem;

		if (data)
		{
			nextFree = data;
			size = n;
			return true;
		}
		return false;
	};

	NEINLINE T * Alloc(int32_t dummy = 0) 
	{
		(void)dummy;
		if (nextFree >= (data + size))
		{
			if (growBy == 0)
				return nullptr;
			
			T * oldData = data;

			if (growBy == -1)
				data = (T*)alloc->Alloc((size * 2) * sizeof(T));
			else
				data = (T*)alloc->Alloc((size + growBy) * sizeof(T));

			if (!data)
			{
				data = oldData;

				return nullptr;
			}

			memcpy(data, oldData, size * sizeof(T));

			if (oldData)
				alloc->Free((neByte*)oldData);

			nextFree = data + size;

			if (growBy == -1)
				size *= 2;
			else
				size += growBy;

			usedSize++;
			return nextFree++;
		}
		else
		{
			usedSize++;
			return nextFree++;
		}
	}
	NEINLINE int32_t GetIndex(T * c)
	{
		ASSERT(c >= data);
		ASSERT(c < nextFree);

		return (int32_t)(c - data);
	}
	NEINLINE int32_t GetUsedCount(){
		return (nextFree - data);
	}
	NEINLINE int32_t GetTotalSize(){
		return size;
	}
	NEINLINE void Free()
	{
		if (IsFixedSize())
		{
			return;
		}
		if (data)
		{
			alloc->Free((neByte*)data);
		}
		data = nullptr;
		nextFree = nullptr;
		size = 0;
		usedSize = 0;
	}
	NEINLINE void Clear()
	{
		nextFree = data;
		usedSize = 0;
	}
	NEINLINE T & operator [] (int32_t index) {
		ASSERT(index >= 0);
		ASSERT(index < size);
		return data[index];
	}
	NEINLINE void MakeFromPointer(T * pdata, int32_t makeSize)
	{
		data = pdata;
		size = makeSize;
		usedSize = size;
		nextFree = pdata + makeSize;
		alloc = nullptr;
		growBy = 0;
	}
protected:
	T * data;
	T * nextFree;
	int32_t size;
	int32_t usedSize;
	neAllocatorAbstract * alloc;
	neAllocatorDefault allocDef;
	int32_t growBy;
	T initArray[initFixedSize];
};

template <class T, int initFixedSize = 1> class neArray
{
PLACEMENT_MAGIC
public:
	NEINLINE bool IsFixedSize()
	{
		return (initFixedSize != 1);
	}
	NEINLINE neArray(){
		if (IsFixedSize())
		{
			data = initArray;
			nextFree = data;
			size = initFixedSize;
			alloc = nullptr;
			growBy = 0;
		}
		else
		{
			data = nullptr;
			nextFree = nullptr;
			size = 0;
			alloc = &allocDef;
			growBy = 0;
		}
	}
	NEINLINE ~neArray()
	{
		Free();
	}
	NEINLINE bool Reserve(int32_t n, neAllocatorAbstract * al = nullptr, int32_t _growBy = 0) 
	{
		if (IsFixedSize())
		{
			ASSERT(0);
			return false;
		}
		Free();

		if (al)
			alloc = al;
		else
			alloc = & allocDef;
		
		growBy = _growBy;

		neByte * mem = alloc->Alloc(sizeof(T) * n);

		data = (T*) mem;

		if (data)
		{
			nextFree = data;
			size = n;
			return true;
		}
		return false;
	};

	NEINLINE T * Alloc() 
	{
		if (nextFree >= (data + size))
		{
			if (growBy == 0)
				return nullptr;
			
			T * oldData = data;

			if (growBy == -1)
				data = (T*)alloc->Alloc(sizeof(T) * (size * 2));
			else
				data = (T*)alloc->Alloc(sizeof(T) * (size + growBy));

			if (!data)
			{
				data = oldData;

				return nullptr;
			}

			memcpy(data, oldData, size * sizeof(T));

			if (oldData)
				alloc->Free((neByte*)oldData);

			nextFree = data + size;

			if (growBy == -1)
				size *= 2;
			else
				size += growBy;

		}
		T * ret = new ((void*)(nextFree++)) T;
		
		return ret;
	}
	NEINLINE int32_t GetIndex(T * c)
	{
		ASSERT(c >= data);
		ASSERT(c < nextFree);

		return (int32_t)(c - data);
	}
	NEINLINE int32_t GetUsedCount(){
		return (nextFree - data);
	}
	NEINLINE int32_t GetTotalSize(){
		return size;
	}
	NEINLINE void Free()
	{
		if (IsFixedSize())
		{
			return;
		}
		if (data)
		{
			//delete [] (data, (void*)data);

			alloc->Free((neByte*)data);
		}
		data = nullptr;
		nextFree = nullptr;
		size = 0;
	}
	NEINLINE void Clear()
	{
		nextFree = data;
	}
	NEINLINE T & operator [] (int32_t index) {
		ASSERT(index >= 0);
		ASSERT(index < size);
		return data[index];
	}
	NEINLINE void MakeFromPointer(T * pdata, int32_t makeSize)
	{
		data = pdata;
		size = makeSize;
		nextFree = pdata + makeSize;
		alloc = nullptr;
		growBy = 0;
	}
protected:
	T * data;
	T * nextFree;
	int32_t size;
	neAllocatorAbstract * alloc;
	neAllocatorDefault allocDef;
	int32_t growBy;

	T initArray[initFixedSize];
};

/////////////////////////////////////////////////////////////

template <class T> class neFreeListItem
{
PLACEMENT_MAGIC
public:
/*
	NEINLINE void * operator new (size_t s,void * addr)
	{
		return addr;
	}
	NEINLINE void * operator new[] (size_t s,void * addr)
	{
		return addr;
	}
	NEINLINE void operator delete(void *, void *)
	{
	}
	NEINLINE void operator delete[](void *, void *)
	{
	}
*/	T thing;
	neFreeListItem * next;
	neFreeListItem * prev;
	bool state;

	NEINLINE neFreeListItem()
	{
		prev = nullptr;
		next = nullptr;
		state = false;
	}

	NEINLINE void Remove()
	{
		if (next != nullptr)
		{
			next->prev = prev;
		}
		if (prev != nullptr)
		{
			prev->next = next;
		}
		Solo();
	}
	NEINLINE void Insert(neFreeListItem * newItem)
	{
		newItem->next = this;
		newItem->prev = prev;
		if (prev)
		{
			prev->next = newItem;
		}
		prev = newItem;
	}
	NEINLINE void Append(neFreeListItem * newItem)
	{
		newItem->next = next;
		newItem->prev = this;
		if (next)
		{
			next->prev = newItem;
		}
		next = newItem;
	}
	NEINLINE void Concat(neFreeListItem * newItem)
	{
		ASSERT(next == nullptr);

		next = newItem;

		newItem->prev = this;
	}
	NEINLINE void Solo()
	{
		prev = nullptr;
		next = nullptr;
	}
};

/////////////////////////////////////////////////////////////

template <class T, int initFixedSize = 1> class neDLinkList
{
public:
	typedef neFreeListItem<T> listItem;

	NEINLINE bool IsFixedSize()
	{
		return (initFixedSize != 1);
	}
	NEINLINE neBool CheckBelongAndInUse(T * t)
	{
		listItem * item = (listItem *)t;

		if (item < data)
			return false;

		if (item >= (data + size))
			return false;

		return item->state; //1 = in use
	}
	NEINLINE void Init()
	{
		if (!IsFixedSize())
		{
			data = nullptr;
			unused = nullptr;
			used = nullptr;
			unusedTail = nullptr;
			usedTail = nullptr;
			size = 0;
			usedCount = 0;
			unusedCount = 0;
		}
		else
		{
			data = initArray;
			size = initFixedSize;
			for (int i = 0; i < size; i++)
			{
				data[i].next = &(data[i+1]);
				data[i].prev = &(data[i-1]);
				data[i].state = false;
			}
			data[0].prev = nullptr;
			data[size-1].next = nullptr;

			unused = data;
			unusedTail = data + size;
			unusedCount = size;

			used = nullptr;
			usedTail = nullptr;
			usedCount = 0;
		}
	}
	NEINLINE neDLinkList()
	{
		alloc = &allocDef;

		Init();
	}
	NEINLINE void Free()
	{
		if (IsFixedSize())
			return;

		//delete [] (data, (void*) data);

		if (data)
			alloc->Free((neByte*)data-mallocNewDiff);

		Init();
	}
	NEINLINE int32_t Size()
	{
		return size;
	}
	NEINLINE ~neDLinkList()
	{
		Free();
	}
	NEINLINE T * Alloc(int32_t flag = 0)
	{
		if (!unused)
			return nullptr;
		
		T * ret = &(unused->thing);

		ASSERT(unused->state == false);

		unused->state = true;

		listItem * newUnusedHead;

		newUnusedHead = unused->next;
		
		unused->Remove();

		if (flag == 0)
		{
			if (usedTail)
			{
				usedTail->Append(unused);
				usedTail = unused;
			}
			else
			{
				used = unused;
				used->Solo();
				usedTail = used;
			}
		}
		else
		{
			unused->Solo();
		}

		if (unused == unusedTail)
		{
			unusedTail = nullptr;
			unused = nullptr;;
			ASSERT(newUnusedHead == nullptr);
		}
		else
			unused = newUnusedHead;

		unusedCount--;
		usedCount++;
		return ret;
	}
	NEINLINE bool Reserve(int32_t n, neAllocatorAbstract * al = nullptr)
	{
		if (IsFixedSize())
		{
			ASSERT(0);
			return false;
		}
		Free();

		if (n == 0)
			return true;

		if (al)
			alloc = al;

		neByte * mem = alloc->Alloc(sizeof(listItem) * n + 4);

		data = new (mem) listItem[n];

		mallocNewDiff = (neByte*)data - mem;
		
		size = n;

		for (int i = 0; i < n; i++)
		{
			data[i].next = &(data[i+1]);
			data[i].prev = &(data[i-1]);
			data[i].state = false;
		}
		data[0].prev = nullptr;
		data[n-1].next = nullptr;

		unused = data;
		unusedTail = data + size;
		unusedCount = n;

		used = nullptr;
		usedTail = nullptr;
		usedCount = 0;

		return true;
	}
	NEINLINE void Dealloc(T * thing, int32_t flag = 0)
	{
		if (!flag && !used)
		{
			ASSERT(0);
			return;
		}

		int32_t n = GetID(thing);

		ASSERT(n >= 0);
		ASSERT(n < size);

		listItem * newUnused = &(data[n]);
		
		ASSERT(newUnused->state == true);

		newUnused->state = false;

		if (flag == 0)
		{
			if (newUnused == used) // dealloc head of used
			{
				used = newUnused->next;
			}
			if (newUnused == usedTail) // dealloc tail of used
			{
				usedTail = newUnused->prev;
			}
		}

		newUnused->Remove();

		if (unused)
		{
			unused->Insert(newUnused);
		}
		else
		{
			newUnused->Solo();
			unusedTail = newUnused;
		}
		unused = newUnused;
		
		unusedCount++;
		usedCount--;
	}
	NEINLINE int32_t GetID(T * t)
	{
		return ((listItem*)t - data);
	}
	NEINLINE int32_t GetUsedCount()
	{
		return usedCount;
	}
	class iterator;

	NEINLINE void Clear()
	{
		iterator iter;

		for (iter = BeginUsed(); iter.Valid();)
		{
			iterator next = BeginNext(iter);

			Dealloc(*iter);

			iter = next;
		}
	}
	NEINLINE iterator BeginUsed()
	{
		iterator iter;
		
		iter.cur = used;

		return iter;
	}
	NEINLINE iterator BeginUnused()
	{
		iterator iter;
		
		iter.cur = unused;

		return iter;
	}
	NEINLINE iterator BeginNext(const iterator & it)
	{
		iterator next;

		next.cur = it.cur->next;

		return next;
	}
	class iterator
	{
	public:
		NEINLINE T * operator * () const
		{
			if (cur)
				return &(cur->thing);
			return nullptr;
		}
		NEINLINE bool operator ++ (int) 
		{
			if (cur)
			{
				cur = cur->next;
				return true;
			}
			return false;
		}
		NEINLINE bool operator -- () 
		{
			if (cur)
			{
				cur = cur->prev;
				return true;
			}
			return false;
		}
		NEINLINE bool Valid()
		{
			return (cur != nullptr);
		}
	public:
		listItem * cur;
	};

public:
	listItem * data;
	listItem * unused;
	listItem * used;
	listItem * unusedTail;
	listItem * usedTail;
	int32_t size;
	int32_t unusedCount;
	int32_t usedCount;
	neAllocatorAbstract * alloc;
	neAllocatorDefault allocDef;
	int32_t mallocNewDiff;
	listItem initArray[initFixedSize];
};
/*
template <class T, int initFixedSize = 1> class neHeap
{
public:
	typedef neDLinkList<T*, initFixedSize> FreeList;
	
	NEINLINE IsFixedSize()
	{
		return (initFixedSize != 1);
	}
	NEINLINE neHeap()
	{
		Init();
	}
	NEINLINE void Init()
	{
		freeList.Init();

		if (IsFixedSize())
		{
			buffer = initArray;
			alloc = nullptr;
		}
		else
		{
			buffer = nullptr;
			alloc = &allocDef;
		}
	}
	NEINLINE ~neHeap()
	{
		Free();
	}
	NEINLINE bool Reserve(int32_t n, neAllocatorAbstract * al = nullptr)
	{
		if (IsFixedSize())
		{
			ASSERT(0);
			return false;
		}
		Free();

		if (!freeList.Reserve(n, al))
			return false;
		
		if (al)
			alloc = al;

		neByte * mem = alloc->Alloc(sizeof(T) * n + 4);

		buffer = new(mem) T[n];
	
		mallocNewDiff = (neByte*)buffer - mem;
		
		if (!buffer)
		{
			Free();
			return false;
		}
		FreeList::iterator it;
		
		int i = 0;
		for (it = freeList.BeginUnused(); it.Valid(); it++)
		{
			(**it) = &(buffer[i]);
			i++;
		}
		return true;
	}
	NEINLINE T * Alloc(int32_t flag = 0)
	{
		T ** pt =  freeList.Alloc(flag);

		new (*pt) T;
		
		if (!pt)
			return nullptr;
		else
			return *pt;
	}
	NEINLINE void Dealloc(T * t, int32_t flag = 0)
	{
		int32_t offset = GetID(t);

		FreeList::listItem * li = freeList.data + offset;

		freeList.Dealloc((T**)li, flag);
	}
	NEINLINE int32_t GetID(T * t)
	{
		return (t - buffer);
	}
	NEINLINE neBool IsInUse(T * t)
	{
		int32_t i = GetID(t);

		ASSERT(i >= 0 && i <freeList.size);

		return freeList.data[i].state;
	}
	NEINLINE void Free()
	{
		if (IsFixedSize())
			return;

		//delete [] (buffer, (void*)buffer);

		if (buffer)
			alloc->Free((neByte*)buffer-mallocNewDiff);

		freeList.Free();

		Init();
	}
	NEINLINE int32_t GetUsedCount()
	{
		return freeList.usedCount;
	}
	NEINLINE int32_t GetUnusedCount()
	{
		return freeList.unusedCount;
	}
	NEINLINE int32_t Size()
	{
		return freeList.size;
	}
	class iterator;

	NEINLINE iterator BeginUsed()
	{
		iterator cur;

		cur.iter = freeList.BeginUsed();

		return cur;
	}
	NEINLINE iterator BeginUnused()
	{
		iterator cur;

		cur.iter = freeList.BeginUnused();

		return cur;
	}
	NEINLINE iterator BeginNext(const iterator & it)
	{
		iterator next;

		next.iter = freeList.BeginNext(it.iter);

		return next;
	}
	class iterator
	{
	public:
		FreeList::iterator iter;

		NEINLINE T * operator * () const
		{
			return (**iter);
		}
		NEINLINE bool operator ++ (int) 
		{
			return (iter++);
		}
		NEINLINE bool operator -- () 
		{
			return (iter--)
		}
		NEINLINE bool Valid()
		{
			return (iter.Valid());
		}
	};

protected:
	T * buffer;
	FreeList freeList;
	neAllocatorAbstract * alloc;
	neAllocatorDefault allocDef;
	int32_t mallocNewDiff;

	T initArray[initFixedSize];
};
*/
template <class T> class neCollection
{
public:
	typedef neFreeListItem<T*> itemType;

	neCollection()
	{
		Reset();
	}
	void Reset()
	{
		headItem = nullptr;
		tailItem = nullptr;
		count = 0;
	}
	void Add(itemType * add)
	{
		ASSERT(add);
		
		if (headItem)
		{
			tailItem->Append(add);

			tailItem = add;
		}
		else
		{
			headItem = add;
			tailItem = add;
		}
		count++;
	}
	void Remove(itemType * rem)
	{
		ASSERT(count > 0);

		ASSERT(rem);

		if (rem == headItem)
		{
			headItem = rem->next;
		}
		if (rem == tailItem)
		{
			tailItem = rem->prev;
		}
		rem->Remove();

		count --;
	}
	itemType * GetHead()
	{
		return headItem;
	}
	itemType * GetNext(itemType * cur)
	{
		return cur->next;
	}
	itemType * GetPrev(itemType * cur)
	{
		return cur->prev;
	}

public:
	neFreeListItem<T*> * headItem;
	neFreeListItem<T*> * tailItem;
	int32_t count;
};

template <class T> class neList
{
public:
	typedef neFreeListItem<T> itemType;

	neList()
	{
		Reset();
	}
	void Reset()
	{
		headItem = nullptr;
		tailItem = nullptr;
		count = 0;
	}
	void Add(T * add)
	{
		ASSERT(add);

		itemType * addItem = (itemType *)add;
		
		if (headItem)
		{
			tailItem->Append(addItem);

			tailItem = addItem;
		}
		else
		{
			headItem = addItem;
			tailItem = addItem;
		}
		count++;
	}
	void AddOrder(T * add)
	{
		ASSERT(add);

		itemType * addItem = (itemType *)add;

		if (!headItem)
		{
			headItem = addItem;

			tailItem = addItem;

			count = 1;

			return;
		}

		itemType * curItem = tailItem;

		neBool done = false;

		while (curItem)
		{
			T * curT = (T *)curItem;

			if (add->Value() <= curT->Value())
			{
				done = true;

				curItem->Append(addItem);

				if (curItem == tailItem)
				{
					tailItem = addItem;
				}
				break;
			}
			curItem = curItem->prev;
		}
		if (!done)
		{
			headItem->Insert(addItem);

			headItem = addItem;
		}
		count++;
	}
	void UpdateOrder(T * u)
	{
		if (count == 1)
			return;

		itemType * uItem = (itemType *) u;

		itemType * cItem;

		neBool done = false;

		if (uItem == tailItem) // move up
		{
			cItem = uItem->prev;

			Remove(u);

			while (cItem)
			{
				if (((T*)cItem)->Value() >= u->Value())
				{
					cItem->Append(uItem);

					if (cItem == tailItem)
					{
						tailItem = uItem;
					}
					done = true;

					break;
				}
				cItem = cItem->prev;
			}
			if (!done)
			{
				headItem->Insert(uItem);

				headItem = uItem;
			}
			count++; // because Remove dec count;
		}
		else if (uItem == headItem) // move down
		{
			cItem = uItem->next;

			Remove(u);

			while (cItem)
			{
				if (((T*)cItem)->Value() <= u->Value())
				{
					cItem->Insert(uItem);

					if (cItem == headItem)
					{
						headItem = uItem;
					}
					done = true;

					break;
				}
				cItem = cItem->next;
			}
			if (!done)
			{
				tailItem->Append(uItem);

				tailItem = uItem;
			}
			count ++;
		}
		else
		{
			itemType * nextItem = uItem->next;

			T * nextT = (T*) nextItem;

			if (u->Value() < nextT->Value())
			{
				//move down
				cItem = nextItem;

				Remove(u);

				while (cItem)
				{
					if (((T*)cItem)->Value() <= u->Value())
					{
						cItem->Insert(uItem);

						done = true;

						break;
					}
					cItem = cItem->next;
				}
				if (!done)
				{
					tailItem->Append(uItem);

					tailItem = uItem;
				}
				count ++;
			}
			else
			{
				//move up
				cItem = uItem->prev;

				Remove(u);

				while (cItem)
				{
					if (((T*)cItem)->Value() >= u->Value())
					{
						cItem->Append(uItem);

						if (cItem == tailItem)
						{
							tailItem = uItem;
						}
						done = true;

						break;
					}
					cItem = cItem->prev;
				}
				if (!done)
				{
					headItem->Insert(uItem);

					headItem = uItem;
				}
				count++; // because Remove dec count;
			}
		}
	}
	void Remove(T * rem)
	{
		ASSERT(count > 0);

		ASSERT(rem);

		itemType * remItem = (itemType *)rem;

		if (remItem == headItem)
		{
			headItem = remItem->next;
		}
		if (remItem == tailItem)
		{
			tailItem = remItem->prev;
		}
		remItem->Remove();

		count --;
	}
	T * GetHead()
	{
		return(T*)headItem;
	}
	T * GetNext(T * cur)
	{
		return (T*)((itemType*)cur)->next;
	}
	T * GetPrev(T * cur)
	{
		return (T*)((itemType*)cur)->prev;
	}

public:
	itemType * headItem;
	itemType * tailItem;
	int32_t count;
};

#endif //CONTAINERS_H
